/*------------------------------------------------------------------------------ */
/* <copyright file="wlan_api.h" company="Atheros"> */
/*    Copyright (c) 2004-2008 Atheros Corporation.  All rights reserved. */
/*  */
/* This program is free software; you can redistribute it and/or modify */
/* it under the terms of the GNU General Public License version 2 as */
/* published by the Free Software Foundation; */
/* */
/* Software distributed under the License is distributed on an "AS */
/* IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or */
/* implied. See the License for the specific language governing */
/* rights and limitations under the License. */
/* */
/* */
/*------------------------------------------------------------------------------ */
/*============================================================================== */
/* This file contains the API for the host wlan module */
/* */
/* Author(s): ="Atheros" */
/*============================================================================== */
#ifndef _HOST_WLAN_API_H_
#define _HOST_WLAN_API_H_

#ifdef __cplusplus
extern "C" {
#endif


struct ieee80211_node_table;
struct ieee80211_frame;

struct ieee80211_common_ie {
    A_UINT16    ie_chan;
    A_UINT8     *ie_tstamp;
    A_UINT8     *ie_ssid;
    A_UINT8     *ie_rates;
    A_UINT8     *ie_xrates;
    A_UINT8     *ie_country;
    A_UINT8     *ie_wpa;
    A_UINT8     *ie_rsn;
    A_UINT8     *ie_wmm;
    A_UINT8     *ie_ath;
    A_UINT16    ie_capInfo;
    A_UINT16    ie_beaconInt;
    A_UINT8     *ie_tim;
    A_UINT8     *ie_chswitch;
    A_UINT8     ie_erp;
    A_UINT8     *ie_wsc;
#ifdef PYXIS_ADHOC
    A_UINT8     *ie_pyxis;
#endif
};

typedef struct bss {
    A_UINT8                      ni_macaddr[6];
    A_UINT8                      ni_snr;
    A_INT16                      ni_rssi;
    struct bss                   *ni_list_next;
    struct bss                   *ni_list_prev;
    struct bss                   *ni_hash_next;
    struct bss                   *ni_hash_prev;
    struct ieee80211_common_ie   ni_cie;
    A_UINT8                     *ni_buf;
    struct ieee80211_node_table *ni_table;
    A_UINT32                     ni_refcnt;
    int                          ni_scangen;

    A_UINT32                     ni_tstamp;
    A_UINT32                     ni_actcnt;
#ifdef OS_ROAM_MANAGEMENT
    A_UINT32                     ni_si_gen;
#endif
} bss_t;

typedef void wlan_node_iter_func(void *arg, bss_t *);

bss_t *wlan_node_alloc(struct ieee80211_node_table *nt, int wh_size);
void wlan_node_free(bss_t *ni);
void wlan_setup_node(struct ieee80211_node_table *nt, bss_t *ni,
                const A_UINT8 *macaddr);
bss_t *wlan_find_node(struct ieee80211_node_table *nt, const A_UINT8 *macaddr);
void wlan_node_reclaim(struct ieee80211_node_table *nt, bss_t *ni);
void wlan_free_allnodes(struct ieee80211_node_table *nt);
void wlan_iterate_nodes(struct ieee80211_node_table *nt, wlan_node_iter_func *f,
                        void *arg);

void wlan_node_table_init(void *wmip, struct ieee80211_node_table *nt);
void wlan_node_table_reset(struct ieee80211_node_table *nt);
void wlan_node_table_cleanup(struct ieee80211_node_table *nt);

A_STATUS wlan_parse_beacon(A_UINT8 *buf, int framelen,
                           struct ieee80211_common_ie *cie);

A_UINT16 wlan_ieee2freq(int chan);
A_UINT32 wlan_freq2ieee(A_UINT16 freq);

void wlan_set_nodeage(struct ieee80211_node_table *nt, A_UINT32 nodeAge);

void
wlan_refresh_inactive_nodes (struct ieee80211_node_table *nt);

bss_t *
wlan_find_Ssidnode (struct ieee80211_node_table *nt, A_UCHAR *pSsid,
                    A_UINT32 ssidLength, A_BOOL bIsWPA2, A_BOOL bMatchSSID);

bss_t *
wlan_find_matching_Ssidnode (struct ieee80211_node_table *nt, A_UCHAR *pSsid,
                    A_UINT32 ssidLength, A_UINT32 dot11AuthMode, A_UINT32 authMode,
                   A_UINT32 pairwiseCryptoType, A_UINT32 grpwiseCryptoTyp);


void
wlan_node_return (struct ieee80211_node_table *nt, bss_t *ni);

bss_t *wlan_node_remove(struct ieee80211_node_table *nt, A_UINT8 *bssid);

#ifdef __cplusplus
}
#endif

#endif /* _HOST_WLAN_API_H_ */
