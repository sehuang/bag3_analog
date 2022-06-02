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

"""This module defines differential resistor loads."""
# TODO: support separate biases
# TODO: support common bias that is not the substrate

from typing import Mapping, Any, Optional, Type, cast

from bag.util.immutable import Param
from bag.design.module import Module
from bag.layout.template import TemplateDB

from pybag.enum import MinLenMode

from xbase.layout.res.base import ResBasePlaceInfo, ResArrayBase

from ...schematic.diff_res import bag3_analog__diff_res


class DiffRes(ResArrayBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        ResArrayBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        return bag3_analog__diff_res

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            pinfo='The ResBasePlaceInfo object.',
            nx_dum='Number of dummies on each side, X direction',
            ny_dum='Number of dummies on each side, Y direction',
            diff='True to have differential resistor, false to have single resistor, True by default.',
            port_layer='Layer for rout and rout_b, top_layer by default',
            bias_node='Common bias terminal of differential resistors, VDD by default',
            double='True to have 2 pairs of differential resistors, False by default.',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(nx_dum=0, ny_dum=0, diff=True, port_layer=-1, bias_node='VDD', double=False)

    def draw_layout(self) -> None:
        pinfo = cast(ResBasePlaceInfo, ResBasePlaceInfo.make_place_info(self.grid, self.params['pinfo']))
        self.draw_base(pinfo)

        # Get hm_layer and vm_layer WireArrays
        warrs, bulk_warrs = self.connect_hm_vm()

        # Connect all dummies
        self.connect_dummies(warrs, bulk_warrs)

        # Connect to bulk
        self.connect_bulk_xm(bulk_warrs)

        unit_params = dict(
            w=pinfo.w_res,
            l=pinfo.l_res,
            intent=pinfo.res_type,
        )
        nx, ny = pinfo.nx, pinfo.ny
        nx_dum: int = self.params['nx_dum']
        ny_dum: int = self.params['ny_dum']
        npar_tot = nx - 2 * nx_dum
        nser = ny - 2 * ny_dum
        diff: bool = self.params['diff']
        double: bool = self.params['double']
        if diff:
            if double:
                if npar_tot % 4 != 0:
                    raise ValueError(f'npar_tot={npar_tot} has to be divisble by 4 when diff=double=True.')
                npar = npar_tot // 4
            else:
                if npar_tot % 2:
                    raise ValueError(f'npar_tot={npar_tot} has to be even when diff={diff}.')
                npar = npar_tot // 2
        else:
            if double:
                raise ValueError('double has to be False when diff=False.')
            npar = npar_tot
        num_dum = nx * ny - npar_tot * nser

        # --- Routing of unit resistors --- #
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xxm_layer = ym_layer + 1

        port_layer: int = self.params['port_layer']
        if port_layer == -1:
            port_layer = pinfo.top_layer

        # make 2 resistors out of all the resistor units
        if diff:
            if double:
                rp_bot, rp_top = self.connect_units(warrs, nx_dum, nx_dum + npar, ny_dum, ny - ny_dum)
                rm_bot, rm_top = self.connect_units(warrs, nx_dum + npar, nx // 2, ny_dum, ny - ny_dum)
                rp_bot2, rp_top2 = self.connect_units(warrs, nx // 2, nx - nx_dum - npar, ny_dum, ny - ny_dum)
                rm_bot2, rm_top2 = self.connect_units(warrs, nx - nx_dum - npar, nx - nx_dum, ny_dum, ny - ny_dum)
                r_bias = self.connect_wires([rp_top, rm_top, rp_top2, rm_top2])[0]
            else:
                rp_bot, rp_top = self.connect_units(warrs, nx_dum, nx // 2, ny_dum, ny - ny_dum)
                rm_bot, rm_top = self.connect_units(warrs, nx // 2, nx - nx_dum, ny_dum, ny - ny_dum)
                rp_bot2, rm_bot2 = None, None
                r_bias = self.connect_wires([rp_top, rm_top])[0]
            align_p = -1
        else:
            rp_bot, rp_top = self.connect_units(warrs, nx_dum, nx - nx_dum, ny_dum, ny - ny_dum)
            rm_bot = None
            rp_bot2, rm_bot2 = None, None
            r_bias = rp_top
            align_p = 0

        # Supply connections on xxm_layer
        sub_type = cast(ResBasePlaceInfo, self.place_info).res_config['sub_type_default']
        # sup_xxm0 = self.connect_via_stack(self.tr_manager, bulk_warrs[vm_layer][0], xxm_layer, 'sup')
        # sup_xxm1 = self.connect_via_stack(self.tr_manager, bulk_warrs[vm_layer][1], xxm_layer, 'sup')
        sup_name = 'VDD' if sub_type == 'ntap' else 'VSS'
        # self.add_pin(sup_name, [bulk_warrs[xm_layer][0], bulk_warrs[xm_layer][1]])

        # connect bot vm_layers to xxm_layer
        mlm_dict_p = {xm_layer: MinLenMode.LOWER, xxm_layer: MinLenMode.LOWER}
        rp = self.connect_via_stack(self.tr_manager, rp_bot, port_layer, 'sig', align_p, 0, mlm_dict_p)
        rout_pin = 'rout<0>' if double else 'rout'
        self.add_pin(rout_pin, rp)
        if diff:
            mlm_dict_m = {xm_layer: MinLenMode.UPPER, xxm_layer: MinLenMode.UPPER}
            if double:
                port_tid = self.tr_manager.get_next_track_obj(rp, 'sig', 'sig', 1)
                rm = self.connect_via_stack(self.tr_manager, rm_bot, port_layer - 1, 'sig', 1, 0, mlm_dict_m)
                rm = self.connect_to_tracks(rm, port_tid)

                rp2 = self.connect_via_stack(self.tr_manager, rp_bot2, port_layer - 1, 'sig', align_p, 0, mlm_dict_p)
                rp2 = self.connect_to_tracks(rp2, port_tid)
                self.add_pin('rout<1>', rp2)
                rm2 = self.connect_via_stack(self.tr_manager, rm_bot2, port_layer, 'sig', 1, 0, mlm_dict_m)
                self.add_pin('rout_b<1>', rm2)
            else:
                rm = self.connect_via_stack(self.tr_manager, rm_bot, port_layer, 'sig', 1, 0, mlm_dict_m)
            routb_pin = 'rout_b<0>' if double else 'rout_b'
            self.add_pin(routb_pin, rm)

        # connect top vm_layers to bias_node
        bias_node: str = self.params['bias_node']
        if bias_node == sup_name:
            self.connect_to_track_wires(r_bias, bulk_warrs[hm_layer][-2])
        else:
            mlm_dict_m = {xm_layer: MinLenMode.UPPER, xxm_layer: MinLenMode.UPPER}
            rm = self.connect_via_stack(self.tr_manager, r_bias, port_layer, 'sig', 0, 0, mlm_dict_m)
            self.add_pin(bias_node, rm)

        self.sch_params = dict(
            load_params=dict(
                unit_params=unit_params,
                nser=nser,
                npar=npar,
            ),
            dum_params=dict(
                unit_params=unit_params,
                nser=1,
                npar=num_dum,
            ),
            sub_type=sub_type,
            diff=diff,
            bias_node=bias_node,
            double=double,
        )


class DiffResVert(ResArrayBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        ResArrayBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        return bag3_analog__diff_res

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            pinfo='The ResBasePlaceInfo object.',
            nx_dum='Number of dummies on each side, X direction',
            ny_dum='Number of dummies on each side, Y direction',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(nx_dum=0, ny_dum=0)

    def draw_layout(self) -> None:
        pinfo = cast(ResBasePlaceInfo, ResBasePlaceInfo.make_place_info(self.grid, self.params['pinfo']))
        self.draw_base(pinfo)

        # Get hm_layer and vm_layer WireArrays
        warrs, bulk_warrs = self.connect_hm_vm()

        # Connect all dummies
        self.connect_dummies(warrs, bulk_warrs)

        unit_params = dict(
            w=pinfo.w_res,
            l=pinfo.l_res,
            intent=pinfo.res_type,
        )
        nx, ny = pinfo.nx, pinfo.ny
        nx_dum: int = self.params['nx_dum']
        ny_dum: int = self.params['ny_dum']
        npar = nx - 2 * nx_dum
        nser_tot = ny - 2 * ny_dum
        assert nser_tot & 1 == 0, f'ny - 2 * ny_dum = {nser_tot} should be even.'
        num_dum = nx * ny - npar * nser_tot

        # --- Routing of unit resistors --- #
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1

        # make 2 resistors out of all the resistor units
        rp_bot, rp_top = self.connect_units(warrs, nx_dum, nx - nx_dum, ny // 2, ny - ny_dum)
        rm_bot, rm_top = self.connect_units(warrs, nx_dum, nx - nx_dum, ny_dum, ny // 2)

        # Supply connections on xm_layer
        sub_type = cast(ResBasePlaceInfo, self.place_info).res_config['sub_type_default']
        sup_top0 = self.connect_via_stack(self.tr_manager, bulk_warrs[vm_layer][0], xm_layer, 'sup')
        sup_top1 = self.connect_via_stack(self.tr_manager, bulk_warrs[vm_layer][1], xm_layer, 'sup')
        sup_name = 'VDD' if sub_type == 'ntap' else 'VSS'
        self.add_pin(sup_name, [sup_top0, sup_top1])

        # connect rout vm_layers to ym_layer
        r_p = self.connect_via_stack(self.tr_manager, rp_bot, ym_layer, 'sig')
        rb_p = self.connect_via_stack(self.tr_manager, rp_top, ym_layer, 'sig')
        sup_ym1 = self.connect_to_track_wires(sup_top1, rb_p)
        self.add_pin('rout', r_p)

        # connect rout_b vm_layers to ym_layer
        r_m = self.connect_via_stack(self.tr_manager, rm_top, ym_layer, 'sig')
        rb_m = self.connect_via_stack(self.tr_manager, rm_bot, ym_layer, 'sig')
        sup_ym0 = self.connect_to_track_wires(sup_top0, rb_m)
        self.add_pin('rout_b', r_m)

        self.add_pin(sup_name, [sup_ym0, sup_ym1])

        self.sch_params = dict(
            load_params=dict(
                unit_params=unit_params,
                nser=nser_tot // 2,
                npar=npar,
            ),
            dum_params=dict(
                unit_params=unit_params,
                nser=1,
                npar=num_dum,
            ),
            sub_type=sub_type,
        )
