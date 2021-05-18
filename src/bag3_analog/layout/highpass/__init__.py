# -*- coding: utf-8 -*-

# SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
# Copyright 2018 Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


"""This module defines various passive high-pass filter generators
"""

from typing import Dict, Set, Any, cast, List, Optional, Tuple

from pybag.enum import RoundMode, Direction, Orientation
from pybag.core import Transform, BBox

from bag.util.immutable import Param
from bag.util.search import BinaryIterator
from bag.design.module import Module
from bag.layout.util import BBox
from bag.layout.routing.base import TrackID, WireArray
from bag.layout.template import TemplateDB

from xbase.layout.res.base import ResBasePlaceInfo, ResArrayBase

from ...schematic.high_pass import bag3_analog__high_pass


class HighPassDiffCore(ResArrayBase):
    """A differential RC high-pass filter.

    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        ResArrayBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Module]:
        return bag3_analog__high_pass

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The ResBasePlaceInfo object.',
            nx_dum='Number of dummies on each side, X direction',
            ny_dum='Number of dummies on each side, Y direction',
            h_unit='total height, in resolution units.',
            sub_type='the substrate type.',
            threshold='the substrate threshold flavor.',
            in_tr_info='Input track info.',
            out_tr_info='Output track info.',
            bias_idx='Bias port index.',
            vdd_tr_info='Supply track info.',
            res_type='Resistor intent',
            res_options='Configuration dictionary for ResArrayBase.',
            cap_spx='Capacitor horizontal separation, in resolution units.',
            cap_spy='Capacitor vertical space from resistor ports, in resolution units.',
            cap_margin='Capacitor space from edge, in resolution units.',
            show_pins='True to show pins.',
            cap_val='Schematic value for analogLib cap',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            cap_val=1e-9,
            # bias_idx=0,
            # vdd_tr_info=None,
            # res_type='standard',
            # res_options=None,
            # cap_spx=0,
            # cap_spy=0,
            # cap_margin=0,
            # half_blk_x=True,
            # show_pins=True,
        )

    def draw_layout(self):
        h_unit = self.params['h_unit']
        sub_type = self.params['sub_type']
        threshold = self.params['threshold']
        in_tr_info = self.params['in_tr_info']
        out_tr_info = self.params['out_tr_info']
        bias_idx = self.params['bias_idx']
        vdd_tr_info = self.params['vdd_tr_info']
        res_type = self.params['res_type']
        res_options = self.params['res_options']
        cap_spx = self.params['cap_spx']
        cap_spy = self.params['cap_spy']
        cap_margin = self.params['cap_margin']
        show_pins = self.params['show_pins']

        # if res_options is None:
        #     my_options = dict(well_end_mode=2)
        #
        # else:
        #     my_options = res_options.copy()
            # my_options['well_end_mode'] = 2
        # find resistor length
        # info = ResArrayBaseInfo(self.grid, sub_type, threshold, top_layer=top_layer,
        #                         res_type=res_type, grid_type=None, ext_dir='y', options=my_options,
        #                         connect_up=True, half_blk_x=half_blk_x, half_blk_y=True)
        #
        # lmin, lmax = info.get_res_length_bounds()
        # bin_iter = BinaryIterator(lmin, lmax, step=2)
        # while bin_iter.has_next():
        #     lcur = bin_iter.get_next()
        #     htot = info.get_place_info(lcur, w_unit, 1, 1)[3]
        #     if htot < h_unit:
        #         bin_iter.save()
        #         bin_iter.up()
        #     else:
        #         bin_iter.down()
        #
        # # draw resistor
        # l_unit = bin_iter.get_last_save()

        # TODO: add methods to find length?
        pinfo = cast(ResBasePlaceInfo,
                     ResBasePlaceInfo.make_place_info(self.grid, self.params['pinfo']))

        # Draw resistor
        self.draw_base(pinfo)
        conn_layer = self.conn_layer
        vm_layer = conn_layer + 1
        xm_layer = vm_layer + 1
        prim_lay_purp = self.tech_cls.tech_info.get_lay_purp_list(conn_layer - 1)[0]

        assert self.top_layer % 2 == 0, "Top layer must be even"

        # connect resistors
        vdd, biasp, biasn, outp_h, outn_h, xl, xr = self.connect_resistors(bias_idx)
        # draw MOM cap
        nser = self.place_info.nx // 2 - self.params['nx_dum']
        tmp = self.draw_mom_cap(nser, xl, xr, cap_spx, cap_spy, cap_margin)
        caplp, capln, caprp, caprn, caplvm, caprvm = tmp
        # Connect caps to resistors
        self.connect_to_track_wires(outp_h, caplvm)
        self.connect_to_track_wires(outn_h, caprvm)

        # connect outputs to horizontal tracks and draw metal resistors
        pin_layer = caplp.layer_id + 1
        assert pin_layer == self.top_layer
        # TODO: Metal resistors should not have the same size
        pidx, nidx, tr_w = in_tr_info
        inp, inn = self.connect_differential_tracks(caplp, caprp, pin_layer, pidx, nidx, width=tr_w)
        _res_w_l, _res_w_h = self.grid.get_wire_bounds(pin_layer, pidx, tr_w)
        res_in_w = _res_w_h - _res_w_l
        tr_lower, tr_upper = inp.lower, inp.upper
        self.add_res_metal_warr(pin_layer, pidx, tr_lower - res_in_w, tr_lower, width=tr_w)
        self.add_res_metal_warr(pin_layer, nidx, tr_lower - res_in_w, tr_lower, width=tr_w)
        inp = self.add_wires(pin_layer, pidx, tr_lower - 2 * res_in_w, tr_lower - res_in_w, width=tr_w)
        inn = self.add_wires(pin_layer, nidx, tr_lower - 2 * res_in_w, tr_lower - res_in_w, width=tr_w)

        pidx, nidx, tr_w = out_tr_info
        outp, outn = self.connect_differential_tracks(capln, caprn, pin_layer, pidx, nidx, track_lower=tr_lower,
                                                      track_upper=tr_upper, width=tr_w)
        _res_w_l, _res_w_h = self.grid.get_wire_bounds(pin_layer, pidx, tr_w)
        res_out_w = _res_w_h - _res_w_l
        tr_lower, tr_upper = outp.lower, outp.upper
        self.add_res_metal_warr(pin_layer, pidx, tr_upper, tr_upper + res_out_w, width=tr_w)
        self.add_res_metal_warr(pin_layer, nidx, tr_upper, tr_upper + res_out_w, width=tr_w)
        outp = self.add_wires(pin_layer, pidx, tr_upper + res_out_w, tr_upper + 2 * res_out_w,
                              width=tr_w)
        outn = self.add_wires(pin_layer, nidx, tr_upper + res_out_w, tr_upper + 2 * res_out_w,
                              width=tr_w)

        # Draw substrate connects at edges
        # TODO: if we don't have substrate taps we need to bias elsewhere?
        if self.has_substrate_port:
            # Connect to conn layer at edges
            conn_tidx_l = self.grid.coord_to_track(conn_layer, self.bound_box.yl, mode=RoundMode.GREATER_EQ)
            conn_tidx_h = self.grid.coord_to_track(conn_layer, self.bound_box.yh, mode=RoundMode.LESS_EQ)
            conn_tid_l = TrackID(conn_layer, conn_tidx_l)
            conn_tid_h = TrackID(conn_layer, conn_tidx_h)
            sub_conn = []
            for xidx in range(self.nx):
                port = self.get_device_port(xidx, 0, "BULK")
                sub_conn.append(self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, port, conn_tid_l))
                sub_conn.append(self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, port, conn_tid_h))
            self.connect_to_track_wires(sub_conn, vdd)

        # connect/export vdd
        # TODO: what if the substrate is not VDD?
        if not vdd_tr_info:
            self.add_pin('VDD_vm', vdd, label='VDD', show=show_pins, connect=True)
        else:
            self.add_pin('VDD_vm', vdd, label='VDD', show=False)
            for tr_info in vdd_tr_info:
                tid = TrackID(xm_layer, tr_info[0], width=tr_info[1])
                self.add_pin('VDD', self.connect_to_tracks(vdd, tid), show=show_pins, connect=True)

        # add pins
        self.add_pin('biasp', biasp, show=show_pins)
        self.add_pin('biasn', biasn, show=show_pins)
        self.add_pin('outp', outp, show=show_pins)
        self.add_pin('outn', outn, show=show_pins)
        self.add_pin('inp', inp, show=show_pins)
        self.add_pin('inn', inn, show=show_pins)

        self._sch_params = dict(
            l=pinfo.l_res,
            w=pinfo.w_res,
            intent=res_type,
            nser=nser,
            ndum=self.params['nx_dum'],
            res_in_info=(pin_layer, res_in_w, res_in_w),
            res_out_info=(pin_layer, res_out_w, res_out_w),
            is_differential=True,
            cap_val=self.params['cap_val']
        )

    def connect_resistors(self, bias_idx) -> Tuple[WireArray, WireArray, WireArray, WireArray, WireArray, int, int]:
        """Connect the resistors in series. bias_idx sets which track to draw the vm bias wire on"""
        nx = self._info.nx
        nx_dum = self.params['nx_dum']
        conn_layer = self.place_info.conn_layer
        vm_layer = self.place_info.conn_layer + 1
        tr_w_conn = self.tr_manager.get_width(conn_layer, 'mid')  # TODO: width
        w_sup_vm = self.tr_manager.get_width(vm_layer, 'sup')
        prim_lay_purp = self.tech_cls.tech_info.get_lay_purp_list(conn_layer - 1)[0]

        biasp = []
        biasn = []
        outp = outn = None

        # Add all dummies ports to biases
        for idx in range(nx_dum):
            biasp.extend(self.get_res_ports(0, idx))
            biasn.extend(self.get_res_ports(0, nx - 1 - idx))

        # Snake together the resistors in series
        for idx in range(nx_dum, nx // 2):
            cpl = self.get_res_ports(0, idx)
            cpr = self.get_res_ports(0, nx - 1 - idx)
            conn_par = (idx - nx_dum) % 2
            # If we're still on the last dummy, add one terminal to the bias
            if idx == nx_dum:
                biasp.append(cpl[1 - conn_par])
                biasn.append(cpr[1 - conn_par])

            if idx == nx // 2 - 1:
                # Get the centermost as the out terminals
                outp = cpl[conn_par]
                outn = cpr[conn_par]
                if isinstance(outp, WireArray):
                    assert outp.layer_id == conn_layer, \
                            "If the port is a warr, we expect it to be at conn_layer. Otherwise, primitive"
                else:
                    # Bring up to conn layer, as WireArrays
                    tidx = self.grid.coord_to_track(conn_layer, outp.ym, mode=RoundMode.NEAREST)
                    outp = self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, outp,
                                                       TrackID(conn_layer, tidx, tr_w_conn))
                    tidx = self.grid.coord_to_track(conn_layer, outn.ym, mode=RoundMode.NEAREST)
                    outn = self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, outn,
                                                       TrackID(conn_layer, tidx, tr_w_conn))
            else:
                # Snake together the resistors in series
                # Assume the upper ports are all aligned vertically, as are the lower powers
                npl = self.get_res_ports(0, idx + 1)
                npr = self.get_res_ports(0, nx - 2 - idx)
                # Code for handling either BBox or WireArray
                if isinstance(npl[conn_par], WireArray):
                    if npl[conn_par].layer_id == conn_layer:
                        self.connect_wires([npl[conn_par], cpl[conn_par]])
                        self.connect_wires([npr[conn_par], cpr[conn_par]])
                    else:
                        # TODO: add code to connect lower layers up
                        raise RuntimeError("Not supported yet")
                else:
                    pairs = [(npl[conn_par], cpl[conn_par]), (npr[conn_par], cpr[conn_par])]
                    for np, cp in pairs:
                        # Get a track to connect the ports
                        tidx = self.grid.coord_to_track(conn_layer, np.ym, mode=RoundMode.NEAREST)
                        assert tidx == self.grid.coord_to_track(
                            conn_layer, cp.ym, mode=RoundMode.NEAREST), "Expected tracks to align"

                        # Connect ports to track
                        tid = TrackID(conn_layer, tidx, tr_w_conn)
                        warr1 = self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, np, tid)
                        warr2 = self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, cp, tid)
                        self.connect_wires([warr1, warr2])

        # connect bias wires to vertical tracks
        t0 = self.grid.find_next_track(vm_layer, self.bound_box.xl, half_track=True, mode=RoundMode.GREATER_EQ)
        t1 = self.grid.find_next_track(vm_layer, self.bound_box.xh, half_track=True, mode=RoundMode.LESS_EQ)
        bp_tid = TrackID(vm_layer, t0 + bias_idx + 1, w_sup_vm)
        bn_tid = TrackID(vm_layer, t1 - bias_idx - 1, w_sup_vm)
        if isinstance(biasp[0], WireArray):
            assert biasp[0].layer_id == conn_layer, \
                "If the port is a warr, we expect it to be at conn_layer. Otherwise, primitive"
            biasp = self.connect_wires(biasp)
            biasn = self.connect_wires(biasn)
            biasp = self.connect_to_tracks(biasp, bp_tid)
            biasn = self.connect_to_tracks(biasn, bn_tid)
        else:
            # Find track ids in the middle of the ports and connect up to conn_layer
            # Since P and N refer to L/R, we need to separately handle T/B.
            ptidx_list = [self.grid.coord_to_track(conn_layer, bp.ym, mode=RoundMode.NEAREST) for bp in biasp]
            ntidx_list = [self.grid.coord_to_track(conn_layer, bn.ym, mode=RoundMode.NEAREST) for bn in biasn]
            bp_hm = [self.connect_bbox_to_tracks(
                Direction.LOWER, prim_lay_purp, bp, TrackID(conn_layer, ptidx, tr_w_conn))
                for ptidx, bp in zip(ptidx_list, biasp)]
            bn_hm = [self.connect_bbox_to_tracks(
                Direction.LOWER, prim_lay_purp, bn, TrackID(conn_layer, ntidx, tr_w_conn))
                for ntidx, bn in zip(ntidx_list, biasn)]
            # Connect up to vm_layer
            biasp = self.connect_to_tracks(bp_hm, bp_tid)
            biasn = self.connect_to_tracks(bn_hm, bn_tid)

        vdd = self.add_wires(vm_layer, t0, biasp.lower, biasp.upper, num=2, pitch=t1 - t0, width=w_sup_vm)
        xl = self.grid.get_wire_bounds(vm_layer, t0 + 2)[1]
        xr = self.grid.get_wire_bounds(vm_layer, t1 - 2)[0]

        return vdd, biasp, biasn, outp, outn, xl, xr

    def draw_mom_cap(self, nser, xl, xr, cap_spx, cap_spy, cap_margin
                     ) -> Tuple[WireArray, WireArray, WireArray, WireArray, WireArray, WireArray]:
        # get port location
        bot_pin, top_pin = self.get_res_ports(0, 0)
        if isinstance(bot_pin, BBox):
            bot_pin_box = bot_pin
            top_pin_box = top_pin
        else:
            bot_pin_box = bot_pin.bound_box
            top_pin_box = top_pin.bound_box
        bnd_box = self.bound_box
        cap_yb_list = [bnd_box.yl + cap_spy, bot_pin_box.yh + cap_spy,
                       top_pin_box.yh + cap_spy]
        cap_yt_list = [bot_pin_box.yl - cap_spy, top_pin_box.yl - cap_spy,
                       bnd_box.yh - cap_spy]

        # draw MOM cap
        xc = bnd_box.xm
        bot_layer = self.place_info.conn_layer
        top_layer = self.place_info.top_layer - 1
        num_layer = top_layer - bot_layer + 1
        # set bottom parity based on number of resistors to avoid via-to-via spacing errors
        if nser % 2 == 0:
            bot_par_list = [True, False, True]
        else:
            bot_par_list = [False, True, False]

        spx_le = self.grid.get_line_end_space(bot_layer, 1)
        spx_le2 = -(-spx_le // 2)
        cap_spx2 = max(cap_spx // 2, spx_le2)
        cap_xl_list = [xl + cap_margin, xc + cap_spx2]
        cap_xr_list = [xc - cap_spx2, xr - cap_margin]

        rects = []
        capp_list, capn_list, capr_list = [], [], []

        # TODO: clean port flipping code
        port_parity = {top_layer: False}
        if top_layer != bot_layer + 1:
            port_parity[bot_layer + 1] = False
        for cap_xl, cap_xr in zip(cap_xl_list, cap_xr_list):
            curp_list, curn_list, curr_list = [], [], []
            for idx, (cap_yb, cap_yt, bot_par) in enumerate(zip(cap_yb_list, cap_yt_list,
                                                                bot_par_list)):
                for layer in range(bot_layer, top_layer):
                    if (layer - bot_layer)  % 2 == 0:
                        port_parity[layer] = bot_par
                cap_box = BBox(cap_xl, cap_yb, cap_xr, cap_yt)
                try:
                    if idx == 1:
                        ports = self.add_mom_cap(cap_box, bot_layer, num_layer,
                                                 port_plow=port_parity, cap_wires_list=rects)
                        # rects.extend(cur_rects[-1])
                    else:
                        ports = self.add_mom_cap(cap_box, bot_layer, num_layer, port_plow=port_parity)
                except ValueError:
                    # This is to catch when we can't fit the momcaps
                    continue

                # Top level ports
                capp, capn = ports[top_layer]
                if len(capp) > 1:
                    curp_list.append(capp[port_parity[top_layer]])
                    curn_list.append(capn[not port_parity[top_layer]])
                else:
                    curp_list.append(capp[0])
                    curn_list.append(capn[0])
                # Connections to the resistors
                capr = ports[bot_layer + 1][1]
                if len(capr) > 1:
                    curr_list.append(capr[not port_parity[bot_layer + 1]])
                else:
                    curr_list.append(capr[0])

            capp_list.append(curp_list)
            capn_list.append(curn_list)
            capr_list.append(curr_list)
            port_parity[top_layer] = True
            port_parity[bot_layer + 1] = True

        caplp = self.connect_wires(capp_list[0])[0]
        caprp = self.connect_wires(capp_list[1])[0]
        capln = self.connect_wires(capn_list[0])[0]
        caprn = self.connect_wires(capn_list[1])[0]
        caplr = self.connect_wires(capr_list[0])[0]
        caprr = self.connect_wires(capr_list[1])[0]

        return caplp, capln, caprp, caprn, caplr, caprr


def merge_all_bboxes(lb: List[BBox]):
    ans = BBox.get_invalid_bbox()
    for box in lb:
        ans = ans.get_merge(box)
    return ans
