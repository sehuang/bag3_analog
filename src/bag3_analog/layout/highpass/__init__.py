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

from typing import Dict, Set, Any, cast, List, Optional

import numbers  # TODO:???

from pybag.enum import RoundMode, Direction, Orientation
from pybag.core import Transform, BBox

from bag.util.immutable import Param
from bag.util.search import BinaryIterator
from bag.design.module import Module
from bag.layout.util import BBox
from bag.layout.routing.base import TrackID, TrackManager, WireArray
from bag.layout.template import TemplateDB

from xbase.layout.res.base import ResBasePlaceInfo, ResArrayBase

# TODO: do we need this?
from xbase.layout.array.top import ArrayBaseWrapper

from ...schematic.high_pass import bag3_analog__high_pass


class HighPassDiffCore(ResArrayBase):
    """A differential RC high-pass filter.

    """

    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        ResArrayBase.__init__(self, temp_db, params, **kwargs)
        self._sch_params = None

    @classmethod
    def get_schematic_class(cls) -> Optional[Module]:
        return bag3_analog__high_pass

    @property
    def sch_params(self) -> Dict[str, Any]:
        return self._sch_params

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
    def get_default_param_values(cls):
        # type: () -> Dict[str, Any]
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

        # connect resistors
        vdd, biasp, biasn, outp_h, outn_h, xl, xr = self.connect_resistors(bias_idx)
        # draw MOM cap
        nser = self.place_info.nx // 2 - self.params['nx_dum']
        caplp, capln, caprp, caprn = self.draw_mom_cap(nser, xl, xr, cap_spx, cap_spy, cap_margin)

        # connect resistors to MOM cap, and draw metal resistors
        vm_layer = 3  # TODO: fix
        # self.connect_to_tracks(outp_h, capln.track_id)
        # self.connect_to_tracks(outn_h, caprn.track_id)
        conn_lp = self.tech_cls.tech_info.get_lay_purp_list(self.place_info.conn_layer)[0]
        self.connect_bbox_to_track_wires(Direction.LOWER, conn_lp, outp_h, capln)
        self.connect_bbox_to_track_wires(Direction.LOWER, conn_lp, outn_h, caprn)

        # connect outputs to horizontal tracks
        xm_layer = vm_layer + 1
        # TODO: need a better defn for this
        pidx, nidx, tr_w = in_tr_info
        inp, inn = self.connect_differential_tracks(caplp, caprp, xm_layer, pidx, nidx, width=tr_w)
        _res_w_l, _res_w_h = self.grid.get_wire_bounds(xm_layer, pidx, tr_w)
        res_in_w = _res_w_h - _res_w_l
        tr_lower, tr_upper = inp.lower, inp.upper
        self.add_res_metal_warr(xm_layer, pidx, tr_lower - res_in_w, tr_lower, width=tr_w)
        self.add_res_metal_warr(xm_layer, nidx, tr_lower - res_in_w, tr_lower, width=tr_w)
        inp = self.add_wires(xm_layer, pidx, tr_lower - 2 * res_in_w, tr_lower - res_in_w, width=tr_w)
        inn = self.add_wires(xm_layer, nidx, tr_lower - 2 * res_in_w, tr_lower - res_in_w, width=tr_w)

        pidx, nidx, tr_w = out_tr_info
        outp, outn = self.connect_differential_tracks(capln, caprn, xm_layer, pidx, nidx, track_lower=tr_lower,
                                                      track_upper=tr_upper, width=tr_w)
        _res_w_l, _res_w_h = self.grid.get_wire_bounds(xm_layer, pidx, tr_w)
        res_out_w = _res_w_h - _res_w_l
        tr_lower, tr_upper = outp.lower, outp.upper
        self.add_res_metal_warr(xm_layer, pidx, tr_upper, tr_upper + res_out_w, width=tr_w)
        self.add_res_metal_warr(xm_layer, nidx, tr_upper, tr_upper + res_out_w, width=tr_w)
        outp = self.add_wires(xm_layer, pidx, tr_upper + res_out_w, tr_upper + 2 * res_out_w,
                              width=tr_w)
        outn = self.add_wires(xm_layer, nidx, tr_upper + res_out_w, tr_upper + 2 * res_out_w,
                              width=tr_w)
        # connect/export vdd
        if vdd_tr_info is None:
            self.add_pin('VDD_vm', vdd, label='VDD:', show=show_pins)
        else:
            self.add_pin('VDD_vm', vdd, label='VDD', show=show_pins)
            for tr_info in vdd_tr_info:
                tid = TrackID(xm_layer, tr_info[0], width=tr_info[1])
                self.add_pin('VDD', self.connect_to_tracks(vdd, tid), show=show_pins)

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
            res_in_info=(xm_layer, res_in_w, res_in_w),
            res_out_info=(xm_layer, res_out_w, res_out_w),
            cap_val=self.params['cap_val']
        )

    def connect_resistors(self, bias_idx):
        """Connect the resistors in series. bias_idx sets which track to draw the vm bias wire on"""
        nx = self._info.nx
        nx_dum = self.params['nx_dum']
        biasp = []
        biasn = []
        outp = outn = None

        # Add all dummies ports to biases
        for idx in range(nx_dum):
            biasp.extend(self.get_res_ports(0, idx))
            biasn.extend(self.get_res_ports(0, nx - 1 - idx))

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
            else:
                # Snake together the resistors in series
                npl = self.get_res_ports(0, idx + 1)
                npr = self.get_res_ports(0, nx - 2 - idx)
                # Code for handling either BBox or WireArray
                if isinstance(npl[conn_par], WireArray):
                    self.connect_wires([npl[conn_par], cpl[conn_par]])
                    self.connect_wires([npr[conn_par], cpr[conn_par]])
                else:
                    # if we have BBox's then we need to merge
                    port_lp = self.tech_cls.tech_info.get_lay_purp_list(self.place_info.conn_layer)[0]
                    # TODO: add vias down
                    self.add_rect(port_lp, npl[conn_par].get_merge(cpl[conn_par]))
                    self.add_rect(port_lp, npr[conn_par].get_merge(cpr[conn_par]))

        # connect bias wires to vertical tracks
        vm_layer = self.place_info.conn_layer + 1
        t0 = self.grid.find_next_track(vm_layer, self.bound_box.xl, half_track=True,
                                       mode=RoundMode.GREATER_EQ)
        t1 = self.grid.find_next_track(vm_layer, self.bound_box.xh, half_track=True,
                                       mode=RoundMode.LESS_EQ)
        bp_tid = TrackID(vm_layer, t0 + bias_idx + 1)
        bn_tid = TrackID(vm_layer, t1 - bias_idx - 1)
        if isinstance(biasp[0], WireArray):
            biasp = self.connect_wires(biasp)
            biasn = self.connect_wires(biasn)
            biasp = self.connect_to_tracks(biasp, bp_tid)
            biasn = self.connect_to_tracks(biasn, bn_tid)
        else:
            port_lp = self.tech_cls.tech_info.get_lay_purp_list(self.place_info.conn_layer)[0]
            bp_vm = [self.connect_bbox_to_tracks(Direction.LOWER, port_lp, bp, bp_tid) for bp in biasp]
            bn_vm = [self.connect_bbox_to_tracks(Direction.LOWER, port_lp, bn, bn_tid) for bn in biasn]
            biasp = self.connect_wires(bp_vm)[0]
            biasn = self.connect_wires(bn_vm)[0]

        vdd = self.add_wires(vm_layer, t0, biasp.lower, biasp.upper, num=2, pitch=t1 - t0)
        xl = self.grid.get_wire_bounds(vm_layer, t0 + 2)[1]
        xr = self.grid.get_wire_bounds(vm_layer, t1 - 2)[0]

        return vdd, biasp, biasn, outp, outn, xl, xr

    def draw_mom_cap(self, nser, xl, xr, cap_spx, cap_spy, cap_margin):
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
        num_layer = 2
        # TODO: fix
        bot_layer = self.tech_cls.tech_info.bot_layer + 1  # Shrug
        top_layer = bot_layer + num_layer - 1
        # top_layer = 3
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
        capp_list, capn_list = [], []

        # TODO: clean port flipping code
        port_parity = {top_layer: False}
        for cap_xl, cap_xr in zip(cap_xl_list, cap_xr_list):
            curp_list, curn_list = [], []
            for idx, (cap_yb, cap_yt, bot_par) in enumerate(zip(cap_yb_list, cap_yt_list,
                                                                bot_par_list)):
                port_parity[bot_layer] = bot_par
                cap_box = BBox(cap_xl, cap_yb, cap_xr, cap_yt)
                try:
                    if idx == 1:
                        ports = self.add_mom_cap(cap_box, bot_layer, num_layer,
                                                            port_plow=port_parity,
                                                            cap_wires_list=rects)
                        # rects.extend(cur_rects[-1])
                    else:
                        ports = self.add_mom_cap(cap_box, bot_layer, num_layer, port_plow=port_parity)
                except ValueError:
                    # This is to catch when we can't fit the momcaps
                    continue
                capp, capn = ports[top_layer]
                curp_list.append(capp[port_parity[top_layer]])
                curn_list.append(capn[not port_parity[top_layer]])

            capp_list.append(curp_list)
            capn_list.append(curn_list)
            port_parity[top_layer] = True

        caplp = self.connect_wires(capp_list[0])[0]
        caprp = self.connect_wires(capp_list[1])[0]
        capln = self.connect_wires(capn_list[0])[0]
        caprn = self.connect_wires(capn_list[1])[0]

        # merge cap wires
        # yb = caplp.lower
        # yt = caplp.upper
        # for rect in rects:
        #     box = BBox(rect.bbox.xl, yb, rect.bbox.xh, yt)
        #     self.add_rect(rect.layer, box)

        # return ports
        return caplp, capln, caprp, caprn


# class HighPassDiff(SubstrateWrapper):
#     """A differential RC high-pass filter with substrate contact.
#
#     Parameters
#     ----------
#     temp_db : TemplateDB
#         the template database.
#     lib_name : str
#         the layout library name.
#     params : Dict[str, Any]
#         the parameter values.
#     used_names : Set[str]
#         a set of already used cell names.
#     **kwargs
#         dictionary of optional parameters.  See documentation of
#         :class:`bag.layout.template.TemplateBase` for details.
#     """
#
#     def __init__(self, temp_db, lib_name, params, used_names, **kwargs):
#         # type: (TemplateDB, str, Dict[str, Any], Set[str], **kwargs) -> None
#         SubstrateWrapper.__init__(self, temp_db, lib_name, params, used_names, **kwargs)
#
#     @classmethod
#     def get_params_info(cls):
#         # type: () -> Dict[str, str]
#         return dict(
#             w='unit resistor width, in meters.',
#             h_unit='total height, in resolution units.',
#             sub_w='Substrate width.',
#             sub_lch='Substrate channel length.',
#             sub_type='the substrate type.',
#             threshold='the substrate threshold flavor.',
#             top_layer='The top layer ID',
#             nser='number of resistors in series in a branch.',
#             ndum='number of dummy resistors.',
#             in_tr_info='Input track info.',
#             out_tr_info='Output track info.',
#             bias_idx='Bias port index.',
#             vdd_tr_info='Supply track info.',
#             res_type='Resistor intent',
#             res_options='Configuration dictionary for ResArrayBase.',
#             cap_spx='Capacitor horizontal separation, in resolution units.',
#             cap_spy='Capacitor vertical space from resistor ports, in resolution units.',
#             cap_margin='Capacitor space from edge, in resolution units.',
#             sub_tr_w='substrate track width in number of tracks.  None for default.',
#             sub_tids='Substrate contact tr_idx/tr_width tuples.',
#             end_mode='substrate end mode flag.',
#             show_pins='True to show pins.',
#             fill_dummy='True to draw dummy fill.',
#         )
#
#     @classmethod
#     def get_default_param_values(cls):
#         # type: () -> Dict[str, Any]
#         return dict(
#             bias_idx=0,
#             vdd_tr_info=None,
#             res_type='standard',
#             res_options=None,
#             cap_spx=0,
#             cap_spy=0,
#             cap_margin=0,
#             sub_tr_w=None,
#             sub_tids=None,
#             end_mode=15,
#             show_pins=True,
#             fill_dummy=True,
#         )
#
#     def draw_layout(self):
#         h_unit = self.params['h_unit']
#         sub_w = self.params['sub_w']
#         sub_lch = self.params['sub_lch']
#         sub_type = self.params['sub_type']
#         threshold = self.params['threshold']
#         top_layer = self.params['top_layer']
#         in_tr_info = self.params['in_tr_info']
#         out_tr_info = self.params['out_tr_info']
#         vdd_tr_info = self.params['vdd_tr_info']
#         res_type = self.params['res_type']
#         sub_tr_w = self.params['sub_tr_w']
#         sub_tids = self.params['sub_tids']
#         end_mode = self.params['end_mode']
#         show_pins = self.params['show_pins']
#         fill_dummy = self.params['fill_dummy']
#
#         # compute substrate contact height, subtract from h_unit
#         bot_end_mode, top_end_mode = self.get_sub_end_modes(end_mode)
#         h_subb = self.get_substrate_height(self.grid, top_layer, sub_lch, sub_w, sub_type,
#                                            threshold, end_mode=bot_end_mode, is_passive=True)
#         h_subt = self.get_substrate_height(self.grid, top_layer, sub_lch, sub_w, sub_type,
#                                            threshold, end_mode=top_end_mode, is_passive=True)
#
#         hm_layer = ResArrayBase.get_port_layer_id(self.grid.tech_info) + 2
#         tr_off = self.grid.find_next_track(hm_layer, h_subb, half_track=True, mode=1,
#                                            unit_mode=True)
#         params = self.params.copy()
#         params['h_unit'] = h_unit - h_subb - h_subt
#         params['in_tr_info'] = (in_tr_info[0] - tr_off, in_tr_info[1] - tr_off, in_tr_info[2])
#         params['out_tr_info'] = (out_tr_info[0] - tr_off, out_tr_info[1] - tr_off, out_tr_info[2])
#         if vdd_tr_info is not None:
#             new_info_list = []
#             for tr_info in vdd_tr_info:
#                 new_info_list.append((tr_info[0] - tr_off, tr_info[1]))
#             params['vdd_tr_info'] = new_info_list
#         self.draw_layout_helper(HighPassDiffCore, params, sub_lch, sub_w, sub_tr_w, sub_type,
#                                 threshold, show_pins, end_mode=end_mode, is_passive=True,
#                                 sub_tids=sub_tids, res_type=res_type)
#
#         # do max space fill
#         if fill_dummy:
#             for lay_id in range(1, hm_layer - 1):
#                 self.do_max_space_fill(lay_id, fill_pitch=1)
#         self.fill_box = self.bound_box


def merge_all_bboxes(lb: List[BBox]):
    ans = BBox.get_invalid_bbox()
    for box in lb:
        ans = ans.get_merge(box)
    return ans
