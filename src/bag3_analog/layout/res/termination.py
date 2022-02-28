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

from typing import Any, Dict, Optional, Type, cast

from pybag.enum import RoundMode, MinLenMode

from bag.util.immutable import Param
from bag.design.module import Module
from bag.layout.routing.base import TrackID
from bag.layout.template import TemplateDB

from xbase.layout.res.base import ResBasePlaceInfo, ResArrayBase

from ...schematic.res_termination import bag3_analog__res_termination


class Termination(ResArrayBase):
    """An array of resistors to be used as a termination.
    Resistors are connected in parallel in the X direction (horizontally),
    series in the Y direction (vertically).

    Use export_mid to use as a differential termination

    Assumption
    - Top layer is an even layer

    TODO: make the series / parallel direction a parameter.
    """
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        ResArrayBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        return bag3_analog__res_termination

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        return dict(
            pinfo='The ResBasePlaceInfo object.',
            nx_dum='Number of dummies on each side, X direction',
            ny_dum='Number of dummies on each side, Y direction',
            export_mid="True to export mid, so this can be used differentially",
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(nx_dum=0, ny_dum=0, export_mid=True)

    def draw_layout(self) -> None:
        pinfo = cast(ResBasePlaceInfo, ResBasePlaceInfo.make_place_info(self.grid, self.params['pinfo']))
        self.draw_base(pinfo)

        assert pinfo.top_layer >= pinfo.conn_layer + 3

        # Get hm_layer and vm_layer WireArrays
        warrs, bulk_warrs = self.connect_hm_vm()

        # Connect all dummies
        self.connect_dummies(warrs, bulk_warrs)

        # Supply connections on xm_layer
        bot_xm, top_xm = self.connect_bulk_xm(bulk_warrs)

        nx_dum: int = self.params['nx_dum']
        ny_dum: int = self.params['ny_dum']
        export_mid: bool = self.params['export_mid']

        # --- Routing of unit resistors --- #
        lower_bot, lower_top = self.connect_units(warrs, nx_dum, pinfo.nx - nx_dum, ny_dum, pinfo.ny // 2)
        upper_bot, upper_top = self.connect_units(warrs, nx_dum, pinfo.nx - nx_dum, pinfo.ny // 2, pinfo.ny - ny_dum)
        pin_list = [('PLUS', upper_top, False)]
        if pinfo.ny == 1:
            pin_list.append(('MINUS', upper_bot, False))
        else:
            pin_list.append(('MINUS', lower_bot, False))
            mid = self.connect_wires([lower_top, upper_bot])[0]
            pin_list.append(('MID', mid, not export_mid))

        # connect to xm_layer
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        w_xm_sig = self.tr_manager.get_width(xm_layer, 'sig')
        for pin_name, warr, _hide in pin_list:
            xm_idx = self.grid.coord_to_track(xm_layer, warr.middle, RoundMode.NEAREST)
            if pin_name == 'PLUS':
                avail_idx = self.tr_manager.get_next_track(xm_layer, top_xm.track_id.base_index, 'sup', 'sig', -1)
                xm_idx = min(xm_idx, avail_idx)
            elif pin_name == 'MINUS':
                avail_idx = self.tr_manager.get_next_track(xm_layer, bot_xm.track_id.base_index, 'sup', 'sig', 1)
                xm_idx = max(xm_idx, avail_idx)
            xm_tid = TrackID(xm_layer, xm_idx, w_xm_sig)
            self.add_pin(pin_name, self.connect_to_tracks(warr, xm_tid, min_len_mode=MinLenMode.MIDDLE), hide=_hide)

        self.sch_params = dict(
            w=pinfo.w_res,
            l=pinfo.l_res,
            res_type=pinfo.res_type,
            nx=pinfo.nx,
            ny=pinfo.ny,
            nx_dum=nx_dum,
            ny_dum=ny_dum,
            export_mid=export_mid,
            sup_name='VDD' if pinfo.res_config['sub_type_default'] == 'ntap' else 'VSS'
        )
