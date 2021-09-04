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

from typing import Dict, Set, Any, cast, List, Optional, Tuple, Mapping

import numpy as np

from pybag.enum import RoundMode, Direction, Orientation, MinLenMode
from pybag.core import Transform, BBox

from bag.util.immutable import Param
from bag.util.search import BinaryIterator
from bag.design.module import Module
from bag.layout.util import BBox
from bag.layout.routing.base import TrackID, WireArray
from bag.layout.template import TemplateDB, TemplateBase

from xbase.layout.res.base import ResBasePlaceInfo, ResArrayBase
from xbase.layout.array.top import ArrayBaseWrapper

from ...schematic.high_pass_diff import bag3_analog__high_pass_diff
from ...schematic.high_pass_array import bag3_analog__high_pass_array
from ...schematic.high_pass_clk import bag3_analog__high_pass_clk
from ...schematic.high_pass_column import bag3_analog__high_pass_column


class HighPassDiffCore(ResArrayBase):
    """A differential RC high-pass filter.

    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        ResArrayBase.__init__(self, temp_db, params, **kwargs)
        self._sup_name = None

    @classmethod
    def get_schematic_class(cls) -> Optional[Module]:
        return bag3_analog__high_pass_diff

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
            bias_idx=0,
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
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        prim_lay_purp = self.tech_cls.tech_info.get_lay_purp_list(conn_layer)[0]

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
            hm_tidx_l = self.grid.coord_to_track(hm_layer, self.bound_box.yl, mode=RoundMode.GREATER_EQ)
            hm_tidx_h = self.grid.coord_to_track(hm_layer, self.bound_box.yh, mode=RoundMode.LESS_EQ)
            hm_tid_l = TrackID(hm_layer, hm_tidx_l)
            hm_tid_h = TrackID(hm_layer, hm_tidx_h)
            sub_conn = []
            for xidx in range(self.nx):
                port = self.get_device_port(xidx, 0, "BULK")
                sub_conn.append(self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, port, hm_tid_l))
                sub_conn.append(self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, port, hm_tid_h))
            self.connect_to_track_wires(sub_conn, vdd)

        # connect/export vdd
        self._sup_name = 'VDD' if \
            cast(ResBasePlaceInfo, self.place_info).res_config['sub_type_default'] == 'ntap' else 'VSS'
        if not vdd_tr_info:
            self.add_pin(f'{self._sup_name}_vm', vdd, label=self._sup_name, show=show_pins, connect=True)
        else:
            self.add_pin(f'{self._sup_name}_vm', vdd, label=self._sup_name, show=False)
            for tr_info in vdd_tr_info:
                tid = TrackID(xm_layer, tr_info[0], width=tr_info[1])
                self.add_pin(self._sup_name, self.connect_to_tracks(vdd, tid), show=show_pins, connect=True)

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
            nser=nser * self.place_info.ny,
            ndum=(self.params['nx_dum'], self.place_info.ny),
            res_in_info=(pin_layer, res_in_w, res_in_w),
            res_out_info=(pin_layer, res_out_w, res_out_w),
            cap_val=self.params['cap_val'],
            sub_name=self._sup_name
        )

    def connect_resistors(self, bias_idx) -> Tuple[WireArray, WireArray, WireArray, WireArray, WireArray, int, int]:
        """Connect the resistors in series. bias_idx sets which track to draw the vm bias wire on"""
        nx = self._info.nx
        ny = self._info.ny
        nx_dum = self.params['nx_dum']
        conn_layer = self.place_info.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        tr_w_hm = self.tr_manager.get_width(hm_layer, 'mid')  # TODO: width
        w_sup_vm = self.tr_manager.get_width(vm_layer, 'sup')
        prim_lay_purp = self.tech_cls.tech_info.get_lay_purp_list(conn_layer)[0]

        biasp = []
        biasn = []
        outp = outn = None

        # Add all dummies ports to biases
        for idx in range(nx_dum):
            biasp.append(self.get_res_ports(0, idx)[0])
            biasp.append(self.get_res_ports(ny - 1, idx)[ny % 2])
            biasn.append(self.get_res_ports(0, nx - 1 - idx)[0])
            biasn.append(self.get_res_ports(ny - 1, nx - 1 - idx)[ny % 2])

        for row in range(ny - 1):
            row_odd = bool(row & 1)
            for col in range(nx):
                bot_res_ports = self.get_res_ports(row, col)
                top_res_ports = self.get_res_ports(row + 1, col)
                if row_odd:
                    bbox_series_conn = bot_res_ports[0].extend(y=top_res_ports[0].yl)
                else:
                    bbox_series_conn = bot_res_ports[1].extend(y=top_res_ports[1].yl)

                self.add_rect(prim_lay_purp, bbox_series_conn)

        # Snake together the resistors in series
        for idx in range(nx_dum, nx // 2):
            idx_rel_dum = idx - nx_dum
            row = 0 if idx_rel_dum % 2 == 0 else ny - 1

            cpl = self.get_res_ports(row, idx)
            cpr = self.get_res_ports(row, nx - 1 - idx)
            conn_par = 0 if idx_rel_dum % 2 == 0 else (1 + row) % 2
            # If we're still on the last dummy, add one terminal to the bias
            if idx == nx_dum:
                tmpl = self.get_res_ports(ny - 1 - row, idx)
                tmpr = self.get_res_ports(ny - 1 - row, nx - 1 - idx)
                biasp.append(tmpl[ny % 2])
                biasn.append(tmpr[ny % 2])

            if idx == nx // 2 - 1:
                # Get the centermost as the out terminals
                outp = cpl[conn_par]
                outn = cpr[conn_par]
                if isinstance(outp, WireArray):
                    raise RuntimeError("WireArray primitives currently not support")
                else:
                    # Bring up to hm_layer, as WireArrays
                    tidx = self.grid.coord_to_track(hm_layer, outp.ym, mode=RoundMode.NEAREST)
                    outp = self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, outp,
                                                       TrackID(hm_layer, tidx, tr_w_hm))
                    tidx = self.grid.coord_to_track(hm_layer, outn.ym, mode=RoundMode.NEAREST)
                    outn = self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, outn,
                                                       TrackID(hm_layer, tidx, tr_w_hm))
            else:
                # Snake together the resistors in series
                # Assume the upper ports are all aligned vertically, as are the lower powers
                npl = self.get_res_ports(row, idx + 1)
                npr = self.get_res_ports(row, nx - 2 - idx)
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
                        tidx = self.grid.coord_to_track(hm_layer, np.ym, mode=RoundMode.NEAREST)
                        assert tidx == self.grid.coord_to_track(
                            hm_layer, cp.ym, mode=RoundMode.NEAREST), "Expected tracks to align"

                        # Connect ports to track
                        tid = TrackID(hm_layer, tidx, tr_w_hm)
                        warr1 = self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, np, tid)
                        warr2 = self.connect_bbox_to_tracks(Direction.LOWER, prim_lay_purp, cp, tid)
                        self.connect_wires([warr1, warr2])

        # connect bias wires to vertical tracks
        t0 = self.grid.find_next_track(vm_layer, self.bound_box.xl, half_track=True, mode=RoundMode.GREATER_EQ)
        t1 = self.grid.find_next_track(vm_layer, self.bound_box.xh, half_track=True, mode=RoundMode.LESS_EQ)
        bp_tid = TrackID(vm_layer, t0 + bias_idx + 1, w_sup_vm)
        bn_tid = TrackID(vm_layer, t1 - bias_idx - 1, w_sup_vm)
        if isinstance(biasp[0], WireArray):
            raise RuntimeError("Currently not supported")
            # biasp = self.connect_wires(biasp)
            # biasn = self.connect_wires(biasn)
            # biasp = self.connect_to_tracks(biasp, bp_tid)
            # biasn = self.connect_to_tracks(biasn, bn_tid)
        else:
            # Find track ids in the middle of the ports and connect up to hm_layer
            # Since P and N refer to L/R, we need to separately handle T/B.
            ptidx_list = [self.grid.coord_to_track(hm_layer, bp.ym, mode=RoundMode.NEAREST) for bp in biasp]
            ntidx_list = [self.grid.coord_to_track(hm_layer, bn.ym, mode=RoundMode.NEAREST) for bn in biasn]
            bp_hm = [self.connect_bbox_to_tracks(
                Direction.LOWER, prim_lay_purp, bp, TrackID(hm_layer, ptidx, tr_w_hm))
                for ptidx, bp in zip(ptidx_list, biasp)]
            bn_hm = [self.connect_bbox_to_tracks(
                Direction.LOWER, prim_lay_purp, bn, TrackID(hm_layer, ntidx, tr_w_hm))
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
        ny = self._info.ny
        bot_pin = self.get_res_ports(0, 0)[0]
        top_pin = self.get_res_ports(ny - 1, 0)[ny % 2]
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
        bot_layer = self.place_info.conn_layer + 1
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
                    if (layer - bot_layer) % 2 == 0:
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


class HighPassArrayCore(ResArrayBase):
    """An row array of RC high-pass filter.
    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        ResArrayBase.__init__(self, temp_db, params, **kwargs)
        self._sup_name = None

    @classmethod
    def get_schematic_class(cls) -> Optional[Module]:
        return bag3_analog__high_pass_array

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The ResBasePlaceInfo object.',
            h_unit='total height, in resolution units.',
            narr='Number of high-pass filters.',
            sub_type='the substrate type.',
            threshold='the substrate threshold flavor.',
            nser='number of resistors in series in a branch.',
            ndum='number of dummy resistors.',
            cap_h_list='List of capacitor heights, in resolution units.',
            port_tr_w='port widths, in number of tracks.',
            res_type='Resistor intent',
            res_options='Configuration dictionary for ResArrayBase.',
            cap_spx='Capacitor horizontal separation, in resolution units.',
            cap_spy='Capacitor vertical margin, in resolution units.',
            show_pins='True to show pins.',
            cap_val='Schematic value for analogLib cap',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            cap_val=1e-9,
            port_tr_w=1,
            res_type='standard',
            res_options=None,
            cap_spx=0,
            cap_spy=0,
        )

    def draw_layout(self):
        h_unit = self.params['h_unit']
        narr = self.params['narr']
        sub_type = self.params['sub_type']
        threshold = self.params['threshold']
        nser = self.params['nser']
        ndum = self.params['ndum']
        port_tr_w = self.params['port_tr_w']
        res_type = self.params['res_type']
        res_options = self.params['res_options']
        cap_spx = self.params['cap_spx']
        cap_spy = self.params['cap_spy']
        cap_h_list = self.params['cap_h_list']
        show_pins = self.params['show_pins']

        if nser % 2 != 0:
            raise ValueError('This generator only supports even nser.')
        if narr > len(cap_h_list):
            raise ValueError('cap_h_list should contain at least narr entries')

        assert self.params['pinfo']['ny'] == 1, "Only ny=1 supported right now"
        assert narr * nser + ndum * 2 == self.params['pinfo']['nx'], "Set nx based on calculation"

        # res = self.grid.resolution
        # lay_unit = self.grid.layout_unit
        # w_unit = int(round(w / lay_unit / res))
        #
        # if res_options is None:
        #     my_options = dict(well_end_mode=2)
        # else:
        #     my_options = res_options.copy()
        #     my_options['well_end_mode'] = 2
        # # find resistor length
        # info = ResArrayBaseInfo(self.grid, sub_type, threshold, top_layer=top_layer,
        #                         res_type=res_type, ext_dir='y', options=my_options,
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

        # draw resistor
        # l_unit = bin_iter.get_last_save()
        # nx = 2 * ndum + narr * nser
        # self.draw_array(l_unit * lay_unit * res, w, sub_type, threshold, nx=nx, ny=1,
        #                 top_layer=top_layer, res_type=res_type, grid_type=None, ext_dir='y',
        #                 options=my_options, connect_up=True, half_blk_x=half_blk_x,
        #                 half_blk_y=True, min_height=h_unit)

        # TODO: add methods to find length?
        pinfo = cast(ResBasePlaceInfo,
                     ResBasePlaceInfo.make_place_info(self.grid, self.params['pinfo']))

        # Draw resistor
        self.draw_base(pinfo)
        conn_layer = self.conn_layer
        hm_layer = conn_layer + 1
        vm_layer = hm_layer + 1
        top_layer = self.top_layer

        # Well tap connection
        self._sup_name = 'VDD' if \
            cast(ResBasePlaceInfo, self.place_info).res_config['sub_type_default'] == 'ntap' else 'VSS'

        # get cap settings
        bot_layer = self.conn_layer + 2
        for lay in range(bot_layer, top_layer + 1):
            # TODO: why?
            if self.grid.get_direction(lay) == 'x':
                cap_spx = max(cap_spx, self.grid.get_line_end_space(lay, 1))

        # connect resistors and draw MOM caps
        tmp = self._connect_resistors(narr, nser, ndum, cap_spx, port_tr_w, show_pins)
        rout_list, cap_x_list = tmp
        tmp = self._draw_mom_cap(cap_x_list, bot_layer, top_layer, cap_spy, cap_h_list,
                                 port_tr_w, show_pins)
        cout_list, ores_info, cres_info = tmp

        # connect bias resistor to cap
        for rout, cout in zip(rout_list, cout_list):
            self.connect_to_track_wires(rout, cout)

        # set schematic parameters
        self._sch_params = dict(
            narr=narr,
            ndum=ndum * 2,
            hp_params=dict(
                l=pinfo.l_res,
                w=pinfo.w_res,
                intent=res_type,
                nser=nser,
                ndum=0,  # no dummies per core HPF
                res_in_info=cres_info,
                res_out_info=ores_info,
                sub_name=self._sup_name,
                cap_val=self.params['cap_val']
            ),
        )

    def _connect_resistors(self, narr, nser, ndum, cap_spx, port_tr_w, show_pins):
        nx = 2 * ndum + narr * nser
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        prim_lp = self.grid.tech_info.get_lay_purp_list(self.conn_layer)[0]

        # connect bulk
        # If we have bulk, connect bulk
        # Assume that the bulk port is on conn_layer and can be simply connected
        # to the conn_layer supply lines.
        bulk_hm = []
        if self.has_substrate_port:
            # Hack to check port layer assumption
            assert self._unit.get_port('BULK').get_single_layer() == prim_lp[0]

            # get_device_port does not return multiple bound boxes. This is work around.
            bulk_bbox_list = self._unit.get_port('BULK').get_pins()
            bulk_hm = [
                TrackID(hm_layer, self.grid.coord_to_track(hm_layer, self.bound_box.yl, mode=RoundMode.GREATER_EQ)),
                TrackID(hm_layer, self.grid.coord_to_track(hm_layer, self.bound_box.yh, mode=RoundMode.LESS_EQ)),
            ]
            for yidx in range(self.ny):
                for xidx in range(nx):
                    for bbox in bulk_bbox_list:
                        bulk_bbox = cast(BBox, bbox).get_transform(self._get_transform(xidx, yidx))
                        self.connect_bbox_to_tracks(Direction.LOWER, prim_lp, bulk_bbox, bulk_hm[-1],
                                                    track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)
                        self.connect_bbox_to_tracks(Direction.LOWER, prim_lp, bulk_bbox, bulk_hm[0],
                                                    track_lower=self.bound_box.xl, track_upper=self.bound_box.xh)

        # connect dummies
        supl_list = []
        supr_list = []
        for idx in range(ndum):
            # Connect ports to hm, then add to list to get connected to supplies
            port_bot = self.get_res_ports(0, idx)
            port_bot_hm = self.connect_port_hm(port_bot)
            supl_list.extend(port_bot_hm)

            port_top = self.get_res_ports(0, nx - 1 - idx)
            port_top_hm = self.connect_port_hm(port_top)
            supr_list.extend(port_top_hm)
        supl_list = self.connect_wires(supl_list)
        supr_list = self.connect_wires(supr_list)
        sup_vm = self._connect_supplies(supl_list, supr_list)
        [self.connect_to_tracks(sup_vm, hm_tid) for hm_tid in bulk_hm]

        # get line-end margin so we can know capacitor horizontal spacing
        hm_sp_le = self.grid.get_line_end_space(hm_layer, 1)
        hm_vext = self.grid.get_via_extensions(Direction.LOWER, hm_layer, 1, 1)[0]
        hm_margin = hm_sp_le + hm_vext
        if isinstance(port_tr_w, Mapping):
            bias_spx = self.grid.get_space(vm_layer, port_tr_w[vm_layer])
        else:
            bias_spx = self.grid.get_space(vm_layer, port_tr_w)

        # get capacitor X interval, connect resistors, and get ports
        out_list = []
        cap_x_list = []
        cap_spx2 = cap_spx // 2
        for res_idx in range(narr):
            rl_idx = ndum + res_idx * nser
            # connect series resistors
            for idx in range(nser - 1):
                conn_par = idx % 2
                rcur_idx = rl_idx + idx
                portl = self.connect_port_hm(self.get_res_ports(0, rcur_idx))
                portr = self.connect_port_hm(self.get_res_ports(0, rcur_idx + 1))
                self.connect_wires([portl[conn_par], portr[conn_par]])

            # record ports
            rr_idx = rl_idx + nser - 1
            xl = self.get_res_bbox(0, rl_idx).xl
            xr = self.get_res_bbox(0, rr_idx).xh
            if res_idx % 2 == 1:
                out_list.append(self.connect_port_hm(self.get_res_ports(0, rl_idx))[1])
                bias = self.connect_port_hm(self.get_res_ports(0, rr_idx))[1]
                bias_tr = self.grid.find_next_track(vm_layer, xr - hm_margin, half_track=True,
                                                    mode=RoundMode.LESS_EQ)
                wl = self.grid.get_wire_bounds(vm_layer, bias_tr, width=1)[0]
                cap_xl = xl + cap_spx2
                cap_xr = min(xr - cap_spx2, wl - bias_spx)
            else:
                out_list.append(self.connect_port_hm(self.get_res_ports(0, rr_idx))[1])
                bias = self.connect_port_hm(self.get_res_ports(0, rl_idx))[1]
                bias_tr = self.grid.find_next_track(vm_layer, xl + hm_margin, half_track=True,
                                                    mode=RoundMode.GREATER_EQ)
                wr = self.grid.get_wire_bounds(vm_layer, bias_tr, width=1)[1]
                cap_xl = max(xl + cap_spx2, wr + bias_spx)
                cap_xr = xr - cap_spx2

            cap_x_list.append((cap_xl, cap_xr))
            bias = self.connect_to_tracks(bias, TrackID(vm_layer, bias_tr),
                                          min_len_mode=RoundMode.GREATER_EQ)
            self.add_pin('bias<%d>' % res_idx, bias, show=show_pins)

        return out_list, cap_x_list

    def _connect_supplies(self, supl_list, supr_list):
        vm_layer = self.conn_layer + 2
        xm_layer = vm_layer + 1

        sup_vm = []
        for sup_list, mode, name in ((supl_list, RoundMode.LESS_EQ, f'{self._sup_name}L'),
                                     (supr_list, RoundMode.GREATER_EQ, f'{self._sup_name}R')):
            xc = sup_list[0].middle
            vm_tr = self.grid.coord_to_track(vm_layer, xc, mode=mode)
            sup = self.connect_to_tracks(sup_list, TrackID(vm_layer, vm_tr))
            sup_vm.append(sup)
            xm_tr = self.grid.coord_to_track(xm_layer, sup.middle)
            sup = self.connect_to_tracks(sup, TrackID(xm_layer, xm_tr), min_len_mode=MinLenMode.MIDDLE)
            self.add_pin(name, sup, connect=True, label=f'{self._sup_name}:')
        return sup_vm

    def _draw_mom_cap(self, cap_x_list, bot_layer, top_layer, cap_spy, cap_h_list,
                      port_tr_w, show_pins):
        # get port location
        bnd_box = self.bound_box
        cap_yt = bnd_box.yh - cap_spy

        # draw MOM cap
        num_layer = top_layer - bot_layer + 1

        out_list = []
        out_res_info = in_res_info = None
        for cap_idx, ((cap_xl, cap_xr), cap_h) in enumerate(zip(cap_x_list, cap_h_list)):
            cap_yb = max(bnd_box.yl + cap_spy, cap_yt - cap_h)
            cap_box = BBox(cap_xl, cap_yb, cap_xr, cap_yt)
            # TODO: clean port flipping code
            parity = cap_idx % 2
            port_par = {}
            for layer in range(bot_layer, top_layer + 1):
                if layer % 2:
                    port_par[layer] = bool(parity)
                else:
                    port_par[layer] = False
            ports = self.add_mom_cap(cap_box, bot_layer, num_layer,
                                     port_widths={top_layer: port_tr_w, bot_layer: port_tr_w},
                                     port_plow=port_par)

            # TODO: document how this works...
            warr_in = ports[top_layer][0][0]
            out = ports[bot_layer][1][1 - parity]

            out_list.append(out)
            # draw output metal resistor and port
            out_port, out_res_info = self._add_metal_res(out, go_up=True)
            self.add_pin('out<%d>' % cap_idx, out_port, show=show_pins)
            # draw clock metal resistor and port
            in_port, in_res_info = self._add_metal_res(warr_in, go_up=False)
            self.add_pin('in<%d>' % cap_idx, in_port, show=show_pins)

        # return ports
        return out_list, out_res_info, in_res_info

    def _add_metal_res(self, warr: WireArray, go_up=True):
        tid = warr.track_id
        tidx = tid.base_index
        lay_id = tid.layer_id
        tr_w = tid.width
        _res_w_l, _res_w_h = self.grid.get_wire_bounds(lay_id, tidx, tr_w)
        res_in_w = _res_w_h - _res_w_l
        if go_up:
            coord = warr.upper
            self.add_res_metal_warr(lay_id, tidx, coord, coord + res_in_w, width=tr_w)
            port = self.add_wires(lay_id, tidx, coord + res_in_w, coord + 2 * res_in_w,
                                  width=tr_w)
        else:
            coord = warr.lower
            self.add_res_metal_warr(lay_id, tidx, coord - res_in_w, coord, width=tr_w)
            port = self.add_wires(lay_id, tidx, coord - 2 * res_in_w, coord - res_in_w,
                                  width=tr_w)

        # scale = self.grid.resolution * self.grid.layout_unit
        scale = 1
        return port, (lay_id, res_in_w * scale, res_in_w * scale)


class HighPassArrayClkCore(TemplateBase):
    """An array of clock RC high-pass filters. Drops down a HPF array and connects all the inputs
    together on a horizontal bar.

    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        super().__init__(temp_db, params, **kwargs)
        self._sup_name = None

    @classmethod
    def get_schematic_class(cls) -> Optional[Module]:
        return bag3_analog__high_pass_clk

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return HighPassArrayCore.get_params_info()

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return HighPassArrayCore.get_default_param_values()

    def draw_layout(self):
        # TODO: I'm not sure this is the right wrapper for this...
        wrap_params = dict(
            cls_name='bag3_analog.layout.highpass.HighPassArrayCore',
            params=self.params
        )
        template: ArrayBaseWrapper = self.new_template(params=wrap_params, temp_cls=ArrayBaseWrapper)
        self._sup_name = template.core._sup_name

        top_layer = template.core.place_info.top_layer
        narr = self.params['narr']

        tr_manager = template.core.tr_manager
        pidx, nidx, y0 = self._place_clock_wires(template.core, narr, top_layer, tr_manager)

        xm_layer = top_layer + 1
        xm_w = tr_manager.get_width(xm_layer, 'clk')

        loc = Transform(0, y0)
        inst = self.add_instance(template, inst_name='XARR', xform=loc)
        bnd_box = inst.bound_box.extend(y=0)
        self.set_size_from_bound_box(xm_layer, bnd_box, round_up=True)
        self.array_box = bnd_box
        self.add_cell_boundary(self.bound_box)

        # re-export/connect clocks
        vssl = inst.get_pin(f'{self._sup_name}L')
        vssl = self.extend_wires(vssl, lower=0)
        vssr = inst.get_pin(f'{self._sup_name}R')
        vssr = self.extend_wires(vssr, upper=bnd_box.xh)
        self.add_pin(f'{self._sup_name}L', vssl, label=f'{self._sup_name}:', connect=True)
        self.add_pin(f'{self._sup_name}R', vssr, label=f'{self._sup_name}:', connect=True)
        clkp_list = []
        clkn_list = []
        for idx in range(narr):
            suf = '<%d>' % idx
            self.reexport(inst.get_port('bias' + suf))
            self.reexport(inst.get_port('out' + suf))
            parity = idx % 4
            if parity == 0 or parity == 3:
                clkp_list.append(inst.get_pin('in' + suf))
            else:
                clkn_list.append(inst.get_pin('in' + suf))
        clkp, clkn = self.connect_differential_tracks(clkp_list, clkn_list, xm_layer, pidx, nidx,
                                                      width=xm_w)
        self.add_pin('clkp', clkp)
        self.add_pin('clkn', clkn)

        self._sch_params = template.sch_params

    def _place_clock_wires(self, template, narr, top_layer, tr_manager):
        """Position the two clock wires below the array. Determine the required y-shift
        in the array to keep the clock wires in the 1st quadrant"""
        yb_min = template.bound_box.yh
        for idx in range(narr):
            yb_min = min(yb_min, template.get_port('in<%d>' % idx).get_pins()[0].lower)

        xm_layer = top_layer + 1
        xm_w = tr_manager.get_width(xm_layer, 'clk')

        pidx = self.grid.find_next_track(xm_layer, yb_min, tr_width=xm_w, mode=RoundMode.LESS_EQ)
        nidx = tr_manager.get_next_track(xm_layer, pidx, 'clk', 'clk', up=False)

        # Version from BAG2. # TODO: Why?
        # edge_tr2 = int(round(2 * nidx)) - xm_w
        # if edge_tr2 < -1:
        #     tr_pitch = self.grid.get_track_pitch(xm_layer)
        #     dy = (edge_tr2 + 1) * tr_pitch // 2
        #     blk_h = self.grid.get_block_size(top_layer)[1]
        #     dy = -(-dy // blk_h) * blk_h
        #     tr_delta = dy / tr_pitch
        #     pidx += tr_delta
        #     nidx += tr_delta
        #     dy = -dy
        if nidx < 0:
            coord0 = self.grid.track_to_coord(xm_layer, nidx)
            tr1 = self.grid.coord_to_track(xm_layer, 0, mode=RoundMode.GREATER)
            coord1 = self.grid.track_to_coord(
                xm_layer, tr1)
            dy = coord1 - coord0
            # Quantize to block pitch
            blk_h = self.grid.get_block_size(top_layer)[1]
            dy = -(-dy // blk_h) * blk_h
            nidx = tr1
            pidx = tr_manager.get_next_track(xm_layer, nidx, 'clk', 'clk')
        else:
            dy = 0

        return pidx, nidx, dy


class HighPassColumn(TemplateBase):
    """A column of differential high-pass RC filters.
    """

    @classmethod
    def get_schematic_class(cls) -> Optional[Module]:
        return bag3_analog__high_pass_column

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        # TODO: can I replace this with HPF core's function?
        return dict(
            pinfo='The ResBasePlaceInfo object.',
            narr='Number of high-pass filters.',
            # w='unit resistor width, in meters.',
            h_unit='total height, in resolution units.',
            # lch='channel length, in meters.',
            # ptap_w='NMOS substrate width, in meters/number of fins.',
            threshold='the substrate threshold flavor.',
            sub_type='the substrate type.',
            # top_layer='The top layer ID',
            nx_dum='Number of dummies on each side, X direction',
            ny_dum='Number of dummies on each side, Y direction',
            in_tr_info='Input track info.',
            out_tr_info='Output track info.',
            vdd_tr_info='Supply track info.',
            # tr_widths='Track width dictionary.',
            # tr_spaces='Track spacing dictionary.',
            res_type='Resistor intent',
            res_options='Configuration dictionary for ResArrayBase.',
            cap_spx='Capacitor horizontal separation, in resolution units.',
            cap_spy='Capacitor vertical space from resistor ports, in resolution units.',
            cap_margin='Capacitor space from edge, in resolution units.',
            cap_val='Schematic value for analogLib cap',
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            cap_val=1e-9,
            res_type='standard',
            sub_type='ptap',
            res_options=None,
            nx_dum=1,
            ny_dum=0,
            cap_spx=0,
            cap_spy=0,
            cap_margin=0,
        )

    def draw_layout(self):
        # rc_params = dict(w=w, h_unit=h_unit, sub_w=ptap_w, sub_lch=lch, sub_type=sub_type,
        #                  threshold=threshold, top_layer=top_layer, nser=nser, ndum=ndum,
        #                  in_tr_info=in_tr_info, out_tr_info=out_tr_info, bias_idx=0,
        #                  vdd_tr_info=vdd_tr_info, res_type=res_type, res_options=res_options,
        #                  cap_spx=cap_spx, cap_spy=cap_spy, cap_margin=cap_margin, end_mode=12,
        #                  sub_tr_w=sub_tr_w, sub_tids=sub_tids, show_pins=False, fill_dummy=fill_dummy,)
        # TODO: is this the best way to wrap stuff?
        rc0_params = dict(
            cls_name='bag3_analog.layout.highpass.HighPassDiffCore',
            params=self.params.copy(),
        )
        master0: ArrayBaseWrapper = self.new_template(params=rc0_params, temp_cls=ArrayBaseWrapper)
        rc1_params = dict(
            cls_name='bag3_analog.layout.highpass.HighPassDiffCore',
            params=self.params.copy(append={'bias_idx': 1}),
        )
        master1: ArrayBaseWrapper = self.new_template(params=rc1_params, temp_cls=ArrayBaseWrapper)
        # fg_sub = master0.fg_sub
        fg_sub = 0  # I think this is gone...

        top_layer = master0.core.top_layer

        # place instances
        if fg_sub > 0:
            raise RuntimeError("Currently not supported")
            # TODO: support
            # end_params = dict(lch=lch, fg=fg_sub, sub_type=sub_type, threshold=threshold,
            #                   top_layer=top_layer, end_mode=0b11, guard_ring_nf=0,
            #                   options=ana_options,)
            # end_master = self.new_template(params=end_params, temp_cls=AnalogBaseEnd)
            # end_row_box = end_master.array_box
            #
            # bot_xform = Transform(0, 0)
            # bot_inst = self.add_instance(end_master, inst_name='XROWB', xform=bot_xform)
            # ycur = end_row_box.top_unit
            # ycur, inst_list = self._place_instances(ycur, master0, master1)
            # ycur += end_row_box.top_unit
            # top_xform = Transform(0, ycur, Orientation.MX)
            # top_inst = self.add_instance(end_master, inst_name='XROWT', xform=top_xform)
            # self.fill_box = inst_list[0].bound_box.merge(inst_list[-1].bound_box)
            # bound_box = bot_inst.bound_box.merge(top_inst.bound_box)
        else:
            _, inst_list = self._place_instances(0, master0, master1)
            self.fill_box = bound_box = inst_list[0].bound_box.merge(inst_list[-1].bound_box)

        vdd_list = []
        vss_list = []
        vdd_vm_list = []
        for idx, inst in enumerate(inst_list):
            suffix = '<%d>' % idx
            for name in inst.port_names_iter():
                if name == 'VDD':
                    vdd_list.extend(inst.port_pins_iter('VDD'))
                elif name == 'VSS':
                    vss_list.extend(inst.port_pins_iter('VSS'))
                elif name == 'VDD_vm':
                    vdd_vm_list.extend(inst.port_pins_iter('VDD_vm'))
                else:
                    self.reexport(inst.get_port(name), net_name=name + suffix)

        vdd_vm_list = self.connect_wires(vdd_vm_list)
        self.add_pin('VDD', vdd_list, label='VDD', connect=True)
        self.add_pin('VSS', vss_list, label='VSS:', connect=True)
        self.add_pin('VDD_vm', vdd_vm_list, label='VDD', connect=True)

        # set size
        self.set_size_from_bound_box(top_layer, bound_box)
        # self.array_box = bound_box

        # set schematic parameters
        self._sch_params = dict(
            narr=self.params['narr'],
            hp_params=master0.sch_params
        )

    def _place_instances(self, ycur, master0, master1):
        inst_list = []
        h_unit = self.params['h_unit']
        h_unit = np.atleast_1d(h_unit)
        assert master0.bound_box.yh == master1.bound_box.yh
        hp_height = master0.bound_box.yh
        assert np.all(h_unit >= hp_height)
        for idx in range(self.params['narr']):
            if idx % 2 == 0:
                master = master0
                orient = Orientation.R0
                dy = 0
            else:
                master = master1
                orient = Orientation.MX
                dy = hp_height
            xform = Transform(0, ycur + dy, orient)
            inst = self.add_instance(master, inst_name='X%d' % idx, xform=xform)
            inst_list.append(inst)
            ycur += h_unit[idx % h_unit.size]

        return ycur, inst_list


def merge_all_bboxes(lb: List[BBox]):
    ans = BBox.get_invalid_bbox()
    for box in lb:
        ans = ans.get_merge(box)
    return ans
