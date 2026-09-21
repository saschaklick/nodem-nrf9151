#include "modem_task.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/tls_credentials.h>
#include <zephyr/net/websocket.h>

#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/pdn.h>
#include <modem/modem_key_mgmt.h>
#include <nrf_modem_at.h>

#include "config_store.h"
#include "log.h"
#include "nodem_task.h"

#define TAG "modem"

/* Not TLS_HOSTNAME/TLS_PORT - those would collide with zephyr/net/socket.h's
 * own TLS_HOSTNAME (a SOL_TLS sockopt name, unrelated to this string). Used
 * as the fallback when config_store's "cloud_host" (control.rs's "#host",
 * matching nodem-esp32's NVS_KEY_CLOUD_HOST) is unset or empty - see
 * get_cloud_host(). */
#define MODEM_TASK_HOSTNAME "docker.ws-ai-call-bot.workers.dev"
#define MODEM_TASK_PORT     "443"
#define TLS_SEC_TAG          42

/* config_store's MAX_STR_LEN (127) + NUL. */
#define CLOUD_HOST_LEN 128

/* The host currently in use, set by step_tls_connect() and reused by
 * step_register() for its request's Host header - both need to agree on
 * exactly the same host, and re-reading config_store independently in each
 * could in principle race a "#host" arriving in between. */
static char g_cloud_host[CLOUD_HOST_LEN];

/* Re-read on every connection attempt (not just once), so a "#host"
 * provisioned while idle takes effect on the very next one. A stored value
 * of "" counts as unset too, same as never having stored one - that's what
 * a bare "#host" (clearing back to the default) actually writes, since
 * config_store has no separate "remove". */
static void get_cloud_host(char *buf, size_t buf_len)
{
	config_get_str("cloud_host", buf, buf_len, "");
	if (buf[0] == '\0') {
		strncpy(buf, MODEM_TASK_HOSTNAME, buf_len - 1);
		buf[buf_len - 1] = '\0';
	}
}

/* Root CA the endpoint's leaf certificate chains to (see /certs) - embedded
 * as a byte array at build time by CMakeLists.txt's generate_inc_file_for_target(). */
static const char ca_cert[] = {
#include "gts-root-r4.pem.inc"
};

BUILD_ASSERT(sizeof(ca_cert) < KB(4), "certificate too large");

enum modem_step {
	MODEM_STEP_INIT = 0,
	MODEM_STEP_SIM,
	MODEM_STEP_PDN,
	MODEM_STEP_TLS,
	MODEM_STEP_REGISTER,
	MODEM_STEP_WEBSOCKET,
	MODEM_STEP_COUNT,
};

static const char *const step_name[MODEM_STEP_COUNT] = {
	[MODEM_STEP_INIT] = "init",
	[MODEM_STEP_SIM] = "sim",
	[MODEM_STEP_PDN] = "pdn",
	[MODEM_STEP_TLS] = "tls",
	[MODEM_STEP_REGISTER] = "reg",
	[MODEM_STEP_WEBSOCKET] = "ws",
};

enum step_state {
	STEP_PENDING = 0,
	STEP_RUNNING,
	STEP_OK,
	STEP_ERROR,
};

struct step_status {
	atomic_t state;
	atomic_t err;
};

static struct step_status g_status[MODEM_STEP_COUNT];
static atomic_t g_tls_fd = ATOMIC_INIT(-1);

/* The websocket id returned by websocket_connect() (distinct from g_tls_fd,
 * the raw socket it wraps - see step_websocket_connect()'s doc comment). */
static atomic_t g_ws_sock = ATOMIC_INIT(-1);

/* Bridges the websocket to nodem_process_command() (see nodem_task.c's
 * nodem_process_ws()) across the thread boundary - websocket_supervise()
 * (this file's own thread) and nodem_task() (a different thread, the only
 * one ever allowed to touch nodem-ffi's runtime/DOM state) each only ever
 * act as a single producer or single consumer of a given ring, which is
 * what makes plain ring_buf safe here without an extra mutex - same
 * precondition uart_task.c's ISR/thread pair already relies on.
 *
 * Two separate rings rather than one shared buffer, and for a reason
 * beyond just direction: routing a reply needs to remember *which*
 * transport a command arrived on, and a single merged byte stream loses
 * that the instant two commands interleave. Keeping this transport's
 * bytes (and nodem_task.c's own line-accumulation buffer for them)
 * entirely separate from uart_task.c's is what preserves that pairing -
 * every reply generated while draining ws_rx_ring is guaranteed to belong
 * on the websocket, because nothing else could have gotten mixed into it.
 *
 * No ring buffer for TX on the UART side (uart_task_write() just blocks
 * synchronously via uart_poll_out(), callable from any thread) - but
 * sending here means calling websocket_send_msg() on g_ws_sock, which only
 * this file's own thread may touch, so a TX ring (drained by
 * websocket_supervise()) is what lets nodem_task() queue a reply without
 * needing to know or care whether a websocket is even connected right
 * now.
 *
 * Sized well past a single websocket_recv_msg() chunk (see
 * MODEM_WS_RECV_BUF_SIZE) rather than just big enough for one - nodem_task()
 * only gets to drain ws_rx_ring once per ~17ms render cycle, in which it
 * also has UART processing, a DOM update, and a display flush to do, so it
 * won't always keep pace with a large "pkg" transfer's incoming rate on
 * every single cycle. The ring absorbs that jitter; 512B (worth roughly
 * one nodem_task cycle at typical LTE-M/NB-IoT throughput) wasn't enough
 * headroom and dropped bytes partway through larger transfers once it
 * stayed full for more than a cycle or two. */
#define MODEM_WS_RING_SIZE 4096
RING_BUF_DECLARE(ws_rx_ring, MODEM_WS_RING_SIZE);
RING_BUF_DECLARE(ws_tx_ring, MODEM_WS_RING_SIZE);

size_t modem_ws_read(uint8_t *buf, size_t max_len)
{
	return ring_buf_get(&ws_rx_ring, buf, max_len);
}

size_t modem_ws_write(const uint8_t *buf, size_t len)
{
	return ring_buf_put(&ws_tx_ring, buf, len);
}

/* Tracks the default PDP context (cid 0) brought up as part of
 * lte_lc_connect() - set/cleared by pdn_event_handler(), consumed by
 * step_pdn_connect() and the post-connect supervisor loop below. */
static atomic_t g_pdn_up = ATOMIC_INIT(0);
static K_SEM_DEFINE(pdn_activated_sem, 0, 1);

static void set_status(enum modem_step step, enum step_state state, int err)
{
	atomic_set(&g_status[step].state, state);
	atomic_set(&g_status[step].err, err);
}

static void pdn_event_handler(uint8_t cid, enum pdn_event event, int reason)
{
	if (cid != 0) {
		return;
	}

	switch (event) {
	case PDN_EVENT_ACTIVATED:
		atomic_set(&g_pdn_up, 1);
		k_sem_give(&pdn_activated_sem);
		break;
	case PDN_EVENT_DEACTIVATED:
	case PDN_EVENT_NETWORK_DETACH:
	case PDN_EVENT_CTX_DESTROYED:
		atomic_set(&g_pdn_up, 0);
		warn(TAG, "pdn event %d on default context", event);
		break;
	case PDN_EVENT_CNEC_ESM:
		warn(TAG, "pdn esm error: %s", pdn_esm_strerror(reason));
		break;
	default:
		break;
	}
}

/* modem_key_mgmt_write() (and friends) fail while the LTE link is active, so
 * this can only run during the offline window step_sim_check() opens up
 * before ever calling lte_lc_connect(). */
static int cert_provision(void)
{
	bool exists;
	int err = modem_key_mgmt_exists(TLS_SEC_TAG, MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN, &exists);

	if (err) {
		return err;
	}

	if (exists) {
		int mismatch = modem_key_mgmt_cmp(TLS_SEC_TAG, MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
						   ca_cert, sizeof(ca_cert));

		if (!mismatch) {
			return 0;
		}

		err = modem_key_mgmt_delete(TLS_SEC_TAG, MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN);
		if (err) {
			return err;
		}
	}

	return modem_key_mgmt_write(TLS_SEC_TAG, MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN, ca_cert,
				     sizeof(ca_cert));
}

/* SIM readiness can only be queried once the modem is in full function mode
 * (CFUN=1) - at CFUN=4 (used below for cert provisioning) the UICC interface
 * isn't up yet and AT+CPIN? reliably comes back with a bare ERROR rather
 * than a real "SIM missing"/"PIN needed" answer. Even after CFUN=1, an `OK`
 * to that command only means the modem accepted it - the SIM's own power-up
 * and ATR aren't instantaneous, so this gives it SIM_SETTLE_DELAY_MS before
 * the first attempt and keeps polling up to SIM_READY_TIMEOUT_MS. */
#define SIM_SETTLE_DELAY_MS  2000
#define SIM_READY_TIMEOUT_MS 15000
#define SIM_POLL_INTERVAL_MS 500

static int step_sim_check(void)
{
	int err = lte_lc_offline();

	if (err) {
		return err;
	}

	err = cert_provision();
	if (err) {
		return err;
	}

	err = lte_lc_normal();
	if (err) {
		return err;
	}

	/* Verbose CME errors, best effort - CFUN transitions reset this, so
	 * it has to be resent after every lte_lc_normal(), not just once. */
	(void)nrf_modem_at_printf("AT+CMEE=1");

	k_msleep(SIM_SETTLE_DELAY_MS);

	int64_t deadline = k_uptime_get() + SIM_READY_TIMEOUT_MS;
	char resp[64];

	while (true) {
		err = nrf_modem_at_cmd(resp, sizeof(resp), "AT+CPIN?");
		if (!err && strstr(resp, "READY")) {
			return 0;
		}

		if (k_uptime_get() >= deadline) {
			return err ? err : -EACCES;
		}

		k_msleep(SIM_POLL_INTERVAL_MS);
	}
}

/* The modem is already in CFUN=1 by this point (step_sim_check() switched it
 * there to query the SIM), so this just blocks until registered - which
 * also activates the default PDP context (cid 0) - pdn_event_handler()
 * reports that activation via g_pdn_up/pdn_activated_sem. */
static int step_pdn_connect(void)
{
	int err = lte_lc_connect();

	if (err) {
		return err;
	}

	if (atomic_get(&g_pdn_up)) {
		return 0;
	}

	return k_sem_take(&pdn_activated_sem, K_SECONDS(10));
}

/* Returns an open, TLS-connected socket fd (offloaded to the modem, hence
 * IPPROTO_TLS_1_2 rather than a host-side TLS stack) on success, or a
 * negative errno. Connects to config_store's "cloud_host" if set (see
 * get_cloud_host()), else MODEM_TASK_HOSTNAME - note the provisioned root CA
 * (see ca_cert above) only actually validates the latter, so pointing at an
 * arbitrary other host will fail the TLS handshake, same as nodem-esp32's
 * own "#host" (it doesn't special-case the CA per host either). */
static int step_tls_connect(void)
{
	get_cloud_host(g_cloud_host, sizeof(g_cloud_host));

	struct addrinfo hints = {
		.ai_family = AF_UNSPEC,
		.ai_socktype = SOCK_STREAM,
	};
	struct addrinfo *res;

	int err = getaddrinfo(g_cloud_host, MODEM_TASK_PORT, &hints, &res);

	if (err) {
		return -EIO;
	}

	int fd = socket(res->ai_family, SOCK_STREAM, IPPROTO_TLS_1_2);

	if (fd < 0) {
		err = -errno;
		freeaddrinfo(res);
		return err;
	}

	sec_tag_t sec_tag_list[] = { TLS_SEC_TAG };
	int verify = TLS_PEER_VERIFY_REQUIRED;

	err = setsockopt(fd, SOL_TLS, TLS_PEER_VERIFY, &verify, sizeof(verify));
	if (!err) {
		err = setsockopt(fd, SOL_TLS, TLS_SEC_TAG_LIST, sec_tag_list,
				  sizeof(sec_tag_list));
	}
	if (!err) {
		err = setsockopt(fd, SOL_TLS, TLS_HOSTNAME, g_cloud_host, strlen(g_cloud_host));
	}
	if (!err) {
		err = connect(fd, res->ai_addr, res->ai_addrlen);
	}

	freeaddrinfo(res);

	if (err) {
		err = -errno;
		close(fd);
		return err;
	}

	return fd;
}

/* Cloud device registration - same backend and wire protocol as
 * nodem-esp32's websocket.rs (ensure_device_credentials()/register_device()):
 * "#reg,<code>,<mac_or_imei>,<name>" request, "#id,<id>,<secret>,<name>"
 * response (confirmed against a live response - a previous version of this
 * comment claimed the response was only two fields, which was actually an
 * artifact of REG_RESP_BUF_SIZE being too small to receive the body at all,
 * not the real response shape). The device id, auth secret (needed for any
 * authenticated call to the cloud from here on), and a server-verified
 * device name (which may differ from the name submitted above, e.g.
 * de-duplicated or normalized) all get persisted to complete the
 * registration permanently - under the same NVS key names nodem-esp32 uses
 * (NVS_KEY_DEVICE_ID/NVS_KEY_DEVICE_SECRET/NVS_KEY_DEVICE_NAME/
 * NVS_KEY_REGISTRATION_CODE in its websocket.rs), so
 * "device_id"/"device_secret"/"device_name"/"reg_code" here, not names of
 * our own invention. control.rs's "#reg" command (over UART) is what
 * provisions "reg_code"/"device_name" into config_store; this just acts on
 * them whenever they're pending - a value of "" (or REG_CODE_REJECTED)
 * means nothing to do, which is the common case and cheap to check. */
#define REG_CODE_LEN            6
#define REG_CODE_REJECTED       "0"
#define REG_NAME_MIN_LEN        3
#define REG_NAME_MAX_LEN        64
#define REG_IMEI_LEN            20
#define REG_ID_LEN              64
#define REG_SECRET_LEN          64
#define REG_VERIFIED_NAME_LEN   128

static bool registration_pending(void)
{
	char code[REG_CODE_LEN + 1];

	config_get_str("reg_code", code, sizeof(code), "");
	return code[0] != '\0' && strcmp(code, REG_CODE_REJECTED) != 0;
}

static bool device_registered(void)
{
	char id[REG_ID_LEN];

	config_get_str("device_id", id, sizeof(id), "");
	return id[0] != '\0';
}

static int get_imei(char *buf, size_t buf_len)
{
	char resp[32];
	int err = nrf_modem_at_cmd(resp, sizeof(resp), "AT+CGSN");

	if (err) {
		return err;
	}

	size_t len = 0;

	while (len < sizeof(resp) && isdigit((unsigned char)resp[len])) {
		len++;
	}

	if (len == 0 || len >= buf_len) {
		return -EINVAL;
	}

	memcpy(buf, resp, len);
	buf[len] = '\0';
	return 0;
}

/* Sends the registration HTTP request over `fd` and parses the "#id,<id>,
 * <secret>,<verified_name>" reply out of the response body. "Connection:
 * close" is intentional, not an oversight - the server closing the socket
 * afterward is what feeds this firmware's existing TLS-drop recovery (see
 * modem_task(), which always reconnects a fresh TLS socket right after this
 * runs) rather than needing separate keep-alive/Content-Length handling
 * here.
 *
 * 1024, not something smaller, because Cloudflare's own Report-To/NEL
 * headers alone run 300+ bytes - a buffer sized just for the small "#id,..."
 * body plus "normal" headers filled up (and thus stopped recv()ing) before
 * ever reaching the blank line separating headers from body, let alone the
 * body itself, which looked exactly like the server having sent no body at
 * all until this was found. */
#define REG_RESP_BUF_SIZE 1024

static int register_http_request(int fd, const char *host, const char *code, const char *imei,
				  const char *name, char *id_out, size_t id_out_len,
				  char *secret_out, size_t secret_out_len, char *verified_name_out,
				  size_t verified_name_out_len)
{
	char body[160];
	int body_len = snprintf(body, sizeof(body), "#reg,%s,%s,%s\r\n", code, imei, name);

	if (body_len < 0 || (size_t)body_len >= sizeof(body)) {
		return -EINVAL;
	}

	/* Sized for a full-length (127-char, config_store's cap) "cloud_host"
	 * plus the rest of the request with room to spare. */
	char req[384];
	int req_len = snprintf(req, sizeof(req),
				"POST /api/devices/register HTTP/1.1\r\n"
				"Host: %s\r\n"
				"Content-Type: text/plain\r\n"
				"Content-Length: %d\r\n"
				"Connection: close\r\n"
				"\r\n",
				host, body_len);

	if (req_len < 0 || (size_t)req_len >= sizeof(req)) {
		return -EINVAL;
	}

	if (send(fd, req, req_len, 0) != req_len || send(fd, body, body_len, 0) != body_len) {
		return -errno;
	}

	static char resp[REG_RESP_BUF_SIZE];
	size_t resp_len = 0;

	while (resp_len < sizeof(resp) - 1) {
		ssize_t n = recv(fd, &resp[resp_len], sizeof(resp) - 1 - resp_len, 0);

		if (n < 0) {
			return -errno;
		}
		if (n == 0) {
			break;
		}
		resp_len += (size_t)n;
	}
	resp[resp_len] = '\0';

	info(TAG, "registration response (%u bytes): %s", (unsigned)resp_len, resp);

	int status = 0;

	if (sscanf(resp, "HTTP/%*d.%*d %d", &status) != 1 || status < 200 || status >= 300) {
		warn(TAG, "registration request failed, http status %d", status);
		return -EIO;
	}

	char *body_start = strstr(resp, "\r\n\r\n");

	if (body_start) {
		body_start += 4;
	}

	/* Logged on its own line, isolated from the header dump above - the
	 * body is what actually matters when "#id," parsing below fails. */
	info(TAG, "registration response body: \"%s\"", body_start ? body_start : "(no body found)");

	char *id_start = body_start ? strstr(body_start, "#id,") : NULL;

	if (!id_start) {
		warn(TAG, "registration response has no \"#id,\" in its body");
		return -EINVAL;
	}

	char *id = id_start + 4;
	char *comma1 = strchr(id, ',');

	if (!comma1) {
		warn(TAG, "registration response has no comma after the id");
		return -EINVAL;
	}
	*comma1 = '\0';

	char *secret = comma1 + 1;
	char *comma2 = strchr(secret, ',');

	if (!comma2) {
		warn(TAG, "registration response has no comma after the secret");
		return -EINVAL;
	}
	*comma2 = '\0';

	char *verified_name = comma2 + 1;
	char *name_end = strpbrk(verified_name, ",\r\n");

	if (name_end) {
		*name_end = '\0';
	}

	strncpy(id_out, id, id_out_len - 1);
	id_out[id_out_len - 1] = '\0';
	strncpy(secret_out, secret, secret_out_len - 1);
	secret_out[secret_out_len - 1] = '\0';
	strncpy(verified_name_out, verified_name, verified_name_out_len - 1);
	verified_name_out[verified_name_out_len - 1] = '\0';

	return 0;
}

/* A registration code in config_store always means "(re)register with
 * this", overriding any already-stored device_id - consumed (cleared on
 * success, replaced with REG_CODE_REJECTED on failure) either way, so a bad
 * or already-used code doesn't retry forever.
 *
 * Always returns 0 (every failure is handled internally by rejecting the
 * code) - the caller (modem_task()) only ever calls this once, right after
 * connecting TLS specifically to make this attempt, so there's nothing left
 * to usefully retry here.
 *
 * `fd` is closed on every exit path below the point it's read - modem_task()
 * only holds a TLS connection open for as long as an actual registration
 * attempt is in progress, so nothing here may leave it open across a
 * failure, or it would sit there consuming bandwidth for no reason until
 * the next retry. */
static int step_register(void)
{
	if (!registration_pending()) {
		return 0;
	}

	char code[REG_CODE_LEN + 1];

	config_get_str("reg_code", code, sizeof(code), "");

	char name[REG_NAME_MAX_LEN + 1];

	config_get_str("device_name", name, sizeof(name), "");
	if (strlen(name) < REG_NAME_MIN_LEN || strlen(name) > REG_NAME_MAX_LEN) {
		warn(TAG, "registration code set but device name invalid, dropping code");
		config_set_str("reg_code", "");
		return 0;
	}

	int fd = (int)atomic_get(&g_tls_fd);

	if (fd < 0) {
		return 0;
	}

	char imei[REG_IMEI_LEN];
	int err = get_imei(imei, sizeof(imei));

	if (!err) {
		char id[REG_ID_LEN];
		char secret[REG_SECRET_LEN];
		char verified_name[REG_VERIFIED_NAME_LEN];

		err = register_http_request(fd, g_cloud_host, code, imei, name, id, sizeof(id),
					     secret, sizeof(secret), verified_name,
					     sizeof(verified_name));
		if (!err) {
			/* Permanent registration is complete once the id,
			 * auth secret, and server-verified device name
			 * (which may differ from what was submitted, e.g.
			 * de-duplicated) are all persisted - overwrite the
			 * tentative "device_name" with the verified one
			 * rather than leaving the locally-submitted value
			 * around. */
			config_set_str("device_id", id);
			config_set_str("device_secret", secret);
			config_set_str("device_name", verified_name);
			config_set_str("reg_code", "");
			info(TAG, "device registered, id %s name %s", id, verified_name);
		}
	}

	close(fd);
	atomic_set(&g_tls_fd, -1);

	if (err) {
		error(TAG, "device registration failed, err %d", err);
		config_set_str("reg_code", REG_CODE_REJECTED);
	}

	return 0;
}

/* The device's live channel to the cloud once registered - same backend and
 * auth scheme as nodem-esp32's websocket_task(): connects to "/ws/device"
 * with the device id/secret from registration sent as "X-Device-Id"/
 * "X-Device-Secret" headers (there, a single EspWebSocketClientConfig::headers
 * string; Zephyr's websocket_request wants one array entry per header
 * instead, each already including its own "\r\n").
 *
 * Layers on top of the raw TLS socket step_tls_connect() already opened
 * (`fd`, still tracked by g_tls_fd) rather than opening a second connection
 * - websocket_connect() returns a *different* id (the "websocket socket")
 * used for websocket_send_msg()/websocket_recv_msg(), but `fd` itself must
 * stay open and untouched for as long as the websocket does (see its doc
 * comment) - both need to be closed on the way out, and neither
 * websocket_disconnect() nor closing the websocket id closes the other, so
 * that's left to modem_task() to do explicitly once done. */
static int step_websocket_connect(void)
{
	int fd = (int)atomic_get(&g_tls_fd);

	if (fd < 0) {
		return -ENOTCONN;
	}

	char id[REG_ID_LEN];
	char secret[REG_SECRET_LEN];

	config_get_str("device_id", id, sizeof(id), "");
	config_get_str("device_secret", secret, sizeof(secret), "");

	static char id_header[16 + REG_ID_LEN];
	static char secret_header[20 + REG_SECRET_LEN];

	snprintf(id_header, sizeof(id_header), "X-Device-Id: %s\r\n", id);
	snprintf(secret_header, sizeof(secret_header), "X-Device-Secret: %s\r\n", secret);

	const char *headers[] = { id_header, secret_header, NULL };
	static uint8_t tmp_buf[512];

	struct websocket_request req = {
		.host = g_cloud_host,
		.url = "/ws/device",
		.optional_headers = headers,
		.tmp_buf = tmp_buf,
		.tmp_buf_len = sizeof(tmp_buf),
	};

	int ws_sock = websocket_connect(fd, &req, 10 * MSEC_PER_SEC, NULL);

	if (ws_sock < 0) {
		/* websocket_connect() failing leaves `fd` open (see the
		 * sample this is modeled on) - it's ours to close, same as
		 * step_tls_connect() failing partway through would be. */
		close(fd);
		atomic_set(&g_tls_fd, -1);
		return ws_sock;
	}

	return ws_sock;
}

/* Matches nodem-esp32's websocket_task() inner loop: a "ping,<seconds>" text
 * frame every MODEM_WS_PING_PERIOD_MS as an application-level keepalive
 * (catches a link that looks "connected" but has silently stopped passing
 * data, which a lower-level TCP/TLS keepalive wouldn't), plus pumping
 * ws_tx_ring/ws_rx_ring (see their doc comment) so nodem_task.c's
 * nodem_process_ws() - running on a different thread - can treat the
 * websocket as just another command transport, the same way it already
 * does uart_task.c's UART.
 *
 * Runs until the connection needs to be re-established for any reason,
 * returning which step modem_task() should resume from - MODEM_STEP_PDN if
 * the PDN link itself dropped, MODEM_STEP_TLS otherwise (a fresh
 * registration code taking priority counts as "otherwise" too: it doesn't
 * get special-cased into MODEM_STEP_REGISTER here, since modem_task()
 * already routes through MODEM_STEP_TLS first regardless and decides from
 * there - see its doc comment). Does not itself close `ws_sock`/g_tls_fd;
 * modem_task() does that once this returns, same reasoning as
 * step_websocket_connect() leaving that to its own caller. */
#define MODEM_WS_PING_PERIOD_MS  10000
#define MODEM_WS_RECV_TIMEOUT_MS 200
/* Bigger than a typical command/reply, so a large "pkg" transfer's data
 * moves in fewer, larger websocket_recv_msg() calls rather than being
 * artificially chopped into many small ones each paying its own call
 * overhead - matched to MODEM_WS_RING_SIZE's own reasoning above. */
#define MODEM_WS_RECV_BUF_SIZE   1024

static enum modem_step websocket_supervise(int ws_sock)
{
	int64_t last_ping = k_uptime_get();

	while (true) {
		if (registration_pending()) {
			info(TAG, "registration code pending, disconnecting websocket");
			return MODEM_STEP_TLS;
		}

		if (!atomic_get(&g_pdn_up)) {
			warn(TAG, "pdn connection lost, reconnecting");
			set_status(MODEM_STEP_PDN, STEP_PENDING, 0);
			return MODEM_STEP_PDN;
		}

		if (k_uptime_get() - last_ping >= MODEM_WS_PING_PERIOD_MS) {
			last_ping = k_uptime_get();

			char ping[32];
			int ping_len = snprintf(ping, sizeof(ping), "ping,%lld",
						 (long long)(k_uptime_get() / MSEC_PER_SEC));

			if (websocket_send_msg(ws_sock, (const uint8_t *)ping, ping_len,
						WEBSOCKET_OPCODE_DATA_TEXT, true, true,
						MSEC_PER_SEC) < 0) {
				warn(TAG, "websocket ping failed, reconnecting");
				return MODEM_STEP_TLS;
			}
		}

		/* Whatever nodem_process_ws() (nodem_task.c) queued via
		 * modem_ws_write() since this last ran - drained here, not
		 * sent directly from nodem_task()'s own thread, since
		 * websocket_send_msg() touches ws_sock, which only this
		 * thread may do (see ws_tx_ring's doc comment above). */
		static uint8_t tx_buf[MODEM_WS_RECV_BUF_SIZE];
		size_t tx_len = ring_buf_get(&ws_tx_ring, tx_buf, sizeof(tx_buf));

		if (tx_len > 0) {
			if (websocket_send_msg(ws_sock, tx_buf, tx_len, WEBSOCKET_OPCODE_DATA_TEXT,
						true, true, MSEC_PER_SEC) < 0) {
				warn(TAG, "websocket send failed, reconnecting");
				return MODEM_STEP_TLS;
			}
		}

		static uint8_t rx_buf[MODEM_WS_RECV_BUF_SIZE];
		uint32_t message_type = 0;
		uint64_t remaining = 0;
		int n = websocket_recv_msg(ws_sock, rx_buf, sizeof(rx_buf), &message_type,
					    &remaining, MODEM_WS_RECV_TIMEOUT_MS);

		if (n == -EAGAIN) {
			continue;
		}
		if (n < 0) {
			warn(TAG, "websocket connection lost, err %d, reconnecting", n);
			return MODEM_STEP_TLS;
		}

		uint32_t queued = ring_buf_put(&ws_rx_ring, rx_buf, (size_t)n);

		if (queued > 0) {
			/* Wakes nodem_task immediately rather than leaving
			 * this data to wait for its next render deadline -
			 * see nodem_task_notify_rx()'s doc comment. */
			nodem_task_notify_rx();
		}

		if (queued != (uint32_t)n) {
			/* Ring buffer full (or too full for all of it) - drop
			 * the remainder rather than block this thread (which
			 * also owns the ping/reconnect logic above);
			 * nodem_process_ws() isn't keeping up. Same tradeoff
			 * uart_isr() makes for the same reason. Logs only the
			 * actual shortfall, not all of `n` - ring_buf_put()
			 * still queues however much did fit. */
			warn(TAG, "ws_rx_ring full, dropping %u of %d bytes",
			     (unsigned)((uint32_t)n - queued), n);
		}
	}
}

#define MODEM_RETRY_DELAY_MS 3000
#define MODEM_POLL_PERIOD_MS 1000

static void modem_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	pdn_default_ctx_cb_reg(pdn_event_handler);

	enum modem_step step = MODEM_STEP_INIT;

	while (true) {
		set_status(step, STEP_RUNNING, 0);

		int err;
		int fd = -1;
		int ws_sock = -1;

		switch (step) {
		case MODEM_STEP_INIT:
			err = nrf_modem_lib_init();
			break;
		case MODEM_STEP_SIM:
			err = step_sim_check();
			break;
		case MODEM_STEP_PDN:
			err = step_pdn_connect();
			break;
		case MODEM_STEP_TLS:
			fd = step_tls_connect();
			err = fd < 0 ? fd : 0;
			break;
		case MODEM_STEP_REGISTER:
			err = step_register();
			break;
		case MODEM_STEP_WEBSOCKET:
			ws_sock = step_websocket_connect();
			err = ws_sock < 0 ? ws_sock : 0;
			break;
		default:
			err = 0;
			break;
		}

		if (err) {
			error(TAG, "%s failed, err %d", step_name[step], err);
			set_status(step, STEP_ERROR, err);
			k_msleep(MODEM_RETRY_DELAY_MS);
			continue;
		}

		set_status(step, STEP_OK, 0);

		switch (step) {
		case MODEM_STEP_TLS:
			atomic_set(&g_tls_fd, fd);
			info(TAG, "tls connected, fd %d", fd);
			/* A fresh registration code always takes priority,
			 * even for an already-registered device (see
			 * step_register()'s doc comment on "(re)register") -
			 * otherwise this connection is for the device's
			 * already-registered live channel. Nothing else opens
			 * MODEM_STEP_TLS (see the MODEM_STEP_PDN case below),
			 * so one of the two is always true here. */
			step = registration_pending() ? MODEM_STEP_REGISTER : MODEM_STEP_WEBSOCKET;
			break;

		case MODEM_STEP_REGISTER:
			/* step_register() always closes the TLS connection
			 * it was given once it's done with it (see its doc
			 * comment) - nothing else uses this connection yet,
			 * so go back to idling at PDN rather than
			 * reconnecting TLS again for no reason. */
			set_status(MODEM_STEP_TLS, STEP_PENDING, 0);
			step = MODEM_STEP_PDN;
			break;

		case MODEM_STEP_WEBSOCKET: {
			atomic_set(&g_ws_sock, ws_sock);
			info(TAG, "websocket connected, sock %d", ws_sock);

			enum modem_step next = websocket_supervise(ws_sock);

			/* Neither websocket_disconnect() nor closing ws_sock
			 * closes the raw TLS socket it wraps (see
			 * step_websocket_connect()'s doc comment) - that's
			 * still whatever g_tls_fd was set to when
			 * MODEM_STEP_TLS last succeeded, read before clearing
			 * it below. */
			int raw_fd = (int)atomic_get(&g_tls_fd);

			websocket_disconnect(ws_sock);
			close(raw_fd);
			atomic_set(&g_ws_sock, -1);
			atomic_set(&g_tls_fd, -1);
			set_status(MODEM_STEP_WEBSOCKET, STEP_PENDING, 0);
			step = next;
			break;
		}

		case MODEM_STEP_PDN:
			/* Deliberately doesn't open TLS here - PDN (LTE
			 * attach) alone doesn't consume ongoing bandwidth, so
			 * it's kept up while idle, but TLS - and thus any
			 * bandwidth use - only happens when there's an actual
			 * reason to: an "#reg"-provisioned registration
			 * attempt, or (once registered) the device's live
			 * websocket channel to the cloud. */
			while (true) {
				k_msleep(MODEM_POLL_PERIOD_MS);

				if (!atomic_get(&g_pdn_up)) {
					warn(TAG, "pdn connection lost, reconnecting");
					set_status(MODEM_STEP_PDN, STEP_PENDING, 0);
					break;
				}

				if (registration_pending() || device_registered()) {
					info(TAG, "connecting");
					step = MODEM_STEP_TLS;
					break;
				}
			}
			break;

		default:
			step++;
			break;
		}
	}
}

static const char *state_name(enum step_state state)
{
	switch (state) {
	case STEP_PENDING:
		return "pending";
	case STEP_RUNNING:
		return "running";
	case STEP_OK:
		return "ok";
	case STEP_ERROR:
		return "error";
	default:
		return "?";
	}
}

size_t modem_status_format(char *buf, size_t buf_len)
{
	int n = snprintf(buf, buf_len,
			  "init=%s(%d) sim=%s(%d) pdn=%s(%d) tls=%s(%d) reg=%s(%d) ws=%s(%d) "
			  "fd=%d wsock=%d",
			  state_name(atomic_get(&g_status[MODEM_STEP_INIT].state)),
			  (int)atomic_get(&g_status[MODEM_STEP_INIT].err),
			  state_name(atomic_get(&g_status[MODEM_STEP_SIM].state)),
			  (int)atomic_get(&g_status[MODEM_STEP_SIM].err),
			  state_name(atomic_get(&g_status[MODEM_STEP_PDN].state)),
			  (int)atomic_get(&g_status[MODEM_STEP_PDN].err),
			  state_name(atomic_get(&g_status[MODEM_STEP_TLS].state)),
			  (int)atomic_get(&g_status[MODEM_STEP_TLS].err),
			  state_name(atomic_get(&g_status[MODEM_STEP_REGISTER].state)),
			  (int)atomic_get(&g_status[MODEM_STEP_REGISTER].err),
			  state_name(atomic_get(&g_status[MODEM_STEP_WEBSOCKET].state)),
			  (int)atomic_get(&g_status[MODEM_STEP_WEBSOCKET].err),
			  (int)atomic_get(&g_tls_fd),
			  (int)atomic_get(&g_ws_sock));

	return n > 0 ? (size_t)n : 0;
}

#define MODEM_STATUS_PERIOD_MS 200 /* 5 Hz */

static void modem_status_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	char buf[224];

	while (true) {
		// modem_status_format(buf, sizeof(buf));
		// info(TAG, "%s", buf);

		k_msleep(MODEM_STATUS_PERIOD_MS);
	}
}

#define MODEM_STACK_SIZE 4096
#define MODEM_PRIORITY   6

#define MODEM_STATUS_STACK_SIZE 1024
#define MODEM_STATUS_PRIORITY   7

K_THREAD_DEFINE(modem_tid, MODEM_STACK_SIZE, modem_task, NULL, NULL, NULL, MODEM_PRIORITY, 0, 0);
K_THREAD_DEFINE(modem_status_tid, MODEM_STATUS_STACK_SIZE, modem_status_task, NULL, NULL, NULL,
		 MODEM_STATUS_PRIORITY, 0, 0);
