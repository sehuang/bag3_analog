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

from typing import Any, Mapping, Optional, Type, cast, Sequence

from pybag.enum import RoundMode, MinLenMode, Orient2D
from pybag.core import Transform, BBox

from bag.util.immutable import Param
from bag.design.module import Module
from bag.layout.routing.base import TrackID, TrackManager, WDictType, SpDictType, WireArray
from bag.layout.template import TemplateDB, TemplateBase

from xbase.layout.res.base import ResBasePlaceInfo, ResArrayBase
from xbase.layout.array.top import ArrayBaseWrapper

from ...schematic.res_termination import bag3_analog__res_termination


class Termination(ResArrayBase):
    """An array of resistors to be used as a termination.
    Resistors are connected in parallel in the X direction (horizontally),
    series in the Y direction (vertically).

    Use export_mid to use as a differential termination

    Assumption
    - Top layer is a horizontal layer

    TODO: make the series / parallel direction a parameter.
    """
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        ResArrayBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        return bag3_analog__res_termination

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            pinfo='The ResBasePlaceInfo object.',
            nx_dum='Number of dummies on each side, X direction',
            ny_dum='Number of dummies on each side, Y direction',
            export_mid='True to export mid for differential termination; False for single ended termination; '
                       'True by default',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
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
        pin_list = [('PLUS', upper_top)]
        if pinfo.ny - 2 * ny_dum == 1:
            pin_list.append(('MINUS', upper_bot))
        else:
            pin_list.append(('MINUS', lower_bot))
            mid = self.connect_wires([lower_top, upper_bot])[0]
            if export_mid:
                pin_list.append(('MID', mid))

        # connect to xm_layer
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        w_xm_sig = self.tr_manager.get_width(xm_layer, 'sig')
        for pin_name, warr in pin_list:
            xm_idx = self.grid.coord_to_track(xm_layer, warr.middle, RoundMode.NEAREST)
            if pin_name == 'PLUS':
                avail_idx = self.tr_manager.get_next_track(xm_layer, top_xm.track_id.base_index, 'sup', 'sig', -1)
                xm_idx = min(xm_idx, avail_idx)
            elif pin_name == 'MINUS':
                avail_idx = self.tr_manager.get_next_track(xm_layer, bot_xm.track_id.base_index, 'sup', 'sig', 1)
                xm_idx = max(xm_idx, avail_idx)
            xm_tid = TrackID(xm_layer, xm_idx, w_xm_sig)
            self.add_pin(pin_name, self.connect_to_tracks(warr, xm_tid, min_len_mode=MinLenMode.MIDDLE))

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


class TerminationTop(TemplateBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        TemplateBase.__init__(self, temp_db, params, **kwargs)
        tr_widths: WDictType = self.params['tr_widths']
        tr_spaces: SpDictType = self.params['tr_spaces']
        self._tr_manager = TrackManager(self.grid, tr_widths, tr_spaces)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        return bag3_analog__res_termination

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            tr_widths='Track width dictionary for TrackManager',
            tr_spaces='Track spaces dictionary for TrackManager',
            term_params='Parameters for Termination',
            port_layer='Top layer for ports',
        )

    def draw_layout(self) -> None:
        term_params: Mapping[str, Any] = self.params['term_params']
        term_master = self.new_template(ArrayBaseWrapper, params=dict(cls_name=Termination.get_qualified_name(),
                                                                      params=term_params))

        port_layer: int = self.params['port_layer']
        pinfo_top_layer = term_master.top_layer
        if port_layer <= pinfo_top_layer:
            raise ValueError(f'Since port_layer={port_layer} is <= pinfo.top_layer={pinfo_top_layer}, use '
                             f'Termination directly instead of TerminationTop.')
        if self.grid.get_direction(port_layer) is Orient2D.y:
            raise ValueError(f'This generator expects port_layer={port_layer} to be horizontal.')

        w_pitch, h_pitch = self.grid.get_size_pitch(port_layer)
        w_term, h_term = self.grid.get_size_pitch(pinfo_top_layer)

        w_blk = -(- term_master.bound_box.w // w_pitch) * w_pitch
        x_term = (w_blk - term_master.bound_box.w) // 2
        h_blk = -(- term_master.bound_box.h // h_pitch) * h_pitch
        y_term = (h_blk - term_master.bound_box.h) // 2
        y_term = -(- y_term // h_term) * h_term
        term_inst = self.add_instance(term_master, xform=Transform(dx=x_term, dy=y_term))

        # get tracks on min(port_layer, pinfo.top_layer + 4)
        tr_layer = min(port_layer, pinfo_top_layer + 4)
        bot_tidx = self.grid.coord_to_track(tr_layer, 0, RoundMode.NEAREST)
        top_tidx = self.grid.coord_to_track(tr_layer, h_blk, RoundMode.NEAREST)

        export_mid = term_inst.has_port('MID')
        if export_mid:
            _list = ['sig', 'sig', 'sig', 'sig', 'sig']
        else:
            _list = ['sig', 'sig', 'sig', 'sig']
        tidx_list = self._tr_manager.spread_wires(tr_layer, _list, bot_tidx, top_tidx, ('sig', 'sig'))
        tr_coords = [self.grid.track_to_coord(tr_layer, _tidx) for _tidx in tidx_list]

        # get supply on tr_layer
        sup_name = term_master.sch_params['sup_name']
        sup_xm = term_inst.get_all_port_pins(sup_name)
        sup_yl = self.connect_via_steps(sup_xm[0], tr_layer, 'sup', [tr_coords[0]])
        sup_yh = self.connect_via_steps(sup_xm[-1], tr_layer, 'sup', [tr_coords[-1]])
        sup_port = self.connect_wires([sup_yl, sup_yh])[0]

        # export PLUS
        plus_port = self.connect_via_steps(term_inst.get_pin('PLUS'), tr_layer, 'sig', [tr_coords[-2]],
                                           mlm_dict={tr_layer-1: MinLenMode.UPPER})
        # export MINUS
        minus_port = self.connect_via_steps(term_inst.get_pin('MINUS'), tr_layer, 'sig', [tr_coords[1]],
                                            mlm_dict={tr_layer-1: MinLenMode.LOWER})
        # export MID
        if export_mid:
            mid_port = self.connect_via_steps(term_inst.get_pin('MID'), tr_layer, 'sig', [tr_coords[2]])
        else:
            mid_port = None

        if port_layer > tr_layer:
            sup_port = self.connect_via_stack(self._tr_manager, sup_port, port_layer - 1, 'sup',
                                              coord_list_o_override=[x_term])
            plus_port = self.connect_via_stack(self._tr_manager, plus_port, port_layer, 'sig',
                                               coord_list_p_override=[h_blk], mlm_dict={port_layer-1: MinLenMode.UPPER})
            minus_port = self.connect_via_stack(self._tr_manager, minus_port, port_layer, 'sig',
                                                coord_list_p_override=[0], mlm_dict={port_layer-1: MinLenMode.LOWER})
            if export_mid:
                mid_port = self.connect_via_stack(self._tr_manager, mid_port, port_layer, 'sig',
                                                  coord_list_p_override=[h_blk // 2], alternate_o=True)

        # add pins
        self.add_pin(sup_name, sup_port)
        self.add_pin('PLUS', plus_port)
        self.add_pin('MINUS', minus_port)
        if export_mid:
            self.add_pin('MID', mid_port)

        # set size
        self.set_size_from_bound_box(port_layer, BBox(0, 0, w_blk, h_blk))

        # get schematic parameters
        self.sch_params = term_master.sch_params

    def connect_via_steps(self, warr_xm: WireArray, top_layer: int, w_type: str, coords: Sequence[int],
                          mlm_dict: Mapping[int, MinLenMode] = None) -> WireArray:
        xm_layer = warr_xm.layer_id
        cur_warr = warr_xm
        for _layer in range(xm_layer + 2, top_layer + 1, 2):
            cur_warr = self.connect_via_stack(self._tr_manager, cur_warr, _layer, w_type, coord_list_p_override=coords,
                                              mlm_dict=mlm_dict)
        return cur_warr
