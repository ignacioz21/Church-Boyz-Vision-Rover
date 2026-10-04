#include "../include/coord.h"
#include "../include/config.h"
#include "../include/comms.h"

static PeerState peer;
static uint32_t last_heard_ms = 0;
static uint32_t last_publish_ms = 0;

void coordReset() {
    peer = PeerState();
    last_heard_ms = 0;
    last_publish_ms = 0;
}

// Mensaje: "C,<id>,<col>,<row>,<theta>,<actividad>,<cubo>,<lleva>,<bloqueado>,<meta col>,<meta row>,<paso col>,<paso row>"
void coordPublish(const Pose &me, Activity activity, CubeColor cube, bool carrying, bool blocked,
                  Point goal, Point waypoint) {
    uint32_t now = millis();
    if (now - last_publish_ms < PEER_PUBLISH_MS && last_publish_ms != 0) return;
    last_publish_ms = now;
    char line[120];
    snprintf(line, sizeof(line), "C,%d,%.2f,%.2f,%.1f,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f",
             ROVER_ID, me.p.col, me.p.row, me.theta, (int)activity, (int)cube, carrying ? 1 : 0, blocked ? 1 : 0,
             goal.col, goal.row, waypoint.col, waypoint.row);
    commsPeerSend(line);
}

const PeerState& coordPeer() {
    String line;
    if (commsPeerPoll(line)) {
        int id, activity, cube, carrying, blocked;
        float col, row, theta, gc, gr, wc, wr;
        if (sscanf(line.c_str(), "C,%d,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f",
                   &id, &col, &row, &theta, &activity, &cube, &carrying, &blocked, &gc, &gr, &wc, &wr) == 12 &&
            id == ROVER_PEER_ID) {
            if (!peer.heard || peer.activity != (Activity)activity) peer.since_ms = millis();
            peer.heard = true;
            peer.pose.p.col = col; peer.pose.p.row = row; peer.pose.theta = theta;
            peer.activity = (Activity)activity;
            peer.cube = (CubeColor)cube;
            peer.carrying = carrying != 0;
            peer.blocked = blocked != 0;
            peer.goal.col = gc; peer.goal.row = gr;
            peer.waypoint.col = wc; peer.waypoint.row = wr;
            last_heard_ms = millis();
        }
    }
    if (peer.heard && millis() - last_heard_ms > PEER_TIMEOUT_MS) peer.heard = false;
    return peer;
}

bool coordPeerClaims(CubeColor c) {
    return peer.heard && peer.cube == c &&
           (peer.activity == ACT_GOING || peer.activity == ACT_CAPTURING ||
            peer.activity == ACT_CARRYING || peer.activity == ACT_RELEASING);
}

bool coordIHavePriority(bool i_carry) {
    if (peer.heard && peer.carrying != i_carry) return i_carry;
    return ROVER_ID < ROVER_PEER_ID;
}
