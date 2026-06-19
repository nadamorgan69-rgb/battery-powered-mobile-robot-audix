const HOTSPOT_HOST = "192.168.4.1";

export class Esp32HttpTransport {
    constructor({ onState, onHistory, onStatus }) {
        this.onState = onState;
        this.onHistory = onHistory;
        this.onStatus = onStatus;
        this.baseUrl = `http://${this.host()}`;
        this.stateTimer = null;
        this.historyTimer = null;
        this.connected = false;
    }

    start() {
        this.pollState();
        this.pollHistory();
        this.stateTimer = window.setInterval(() => this.pollState(), 250);
        this.historyTimer = window.setInterval(() => this.pollHistory(), 1000);
    }

    stop() {
        if (this.stateTimer !== null) {
            window.clearInterval(this.stateTimer);
            this.stateTimer = null;
        }
        if (this.historyTimer !== null) {
            window.clearInterval(this.historyTimer);
            this.historyTimer = null;
        }
    }

    async send(command) {
        if (!command) return false;
        try {
            const url = `${this.baseUrl}/api/command?line=${encodeURIComponent(command)}`;
            const response = await fetch(url, { cache: "no-store" });
            if (!response.ok) {
                this.setConnected(false, `command failed ${response.status}`);
                return false;
            }
            await this.pollState();
            await this.pollHistory();
            return true;
        } catch {
            this.setConnected(false, "offline");
            return false;
        }
    }

    async pollState() {
        try {
            const response = await fetch(`${this.baseUrl}/api/state`, { cache: "no-store" });
            if (!response.ok) {
                this.setConnected(false, `state ${response.status}`);
                return;
            }
            const state = await response.json();
            this.setConnected(true, "connected");
            this.onState?.(state);
        } catch {
            this.setConnected(false, "offline");
        }
    }

    async pollHistory() {
        try {
            const response = await fetch(`${this.baseUrl}/api/history`, { cache: "no-store" });
            if (!response.ok) return;
            this.onHistory?.(await response.text());
        } catch {
            // State polling already owns the visible connection status.
        }
    }

    url() {
        return this.baseUrl;
    }

    host() {
        const hostname = window.location.hostname;
        if (!hostname || hostname === "localhost" || hostname === "127.0.0.1") {
            return HOTSPOT_HOST;
        }
        return hostname;
    }

    setConnected(connected, label) {
        if (this.connected === connected && !label) return;
        this.connected = connected;
        this.onStatus?.({ connected, label });
    }
}
