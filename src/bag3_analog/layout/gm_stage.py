"""This module contains layout generator for gm_stage."""

from typing import Any, Mapping, Type, Optional, Sequence

from pybag.enum import MinLenMode, RoundMode

from bag.util.immutable import Param
from bag.layout.template import TemplateDB
from bag.design.module import Module

from xbase.layout.enum import MOSWireType, MOSPortType, SubPortMode
from xbase.layout.mos.base import MOSBasePlaceInfo, MOSBase
from xbase.layout.mos.guardring import GuardRing

from ..schematic.gm_stage import bag3_analog__gm_stage


class GmStage(MOSBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        MOSBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        return bag3_analog__gm_stage

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        return dict(
            pinfo='The MOSBasePlaceInfo object.',
            seg_dict='Dictionary of transistor segments',
            w='transistor width.',
            ridx_tail='index for mos row with tail transistor',
            ridx_gm='index for mos row with gm transistors',
            export_tail='True to export tail node; False by default',
            is_dum='True if the gm stage is used as dummy load; False by default',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(export_tail=False, w=0, ridx_tail=0, ridx_gm=1, is_dum=False)

    def draw_layout(self) -> None:
        pinfo = MOSBasePlaceInfo.make_place_info(self.grid, self.params['pinfo'])
        self.draw_base(pinfo)

        tile_tap = 0
        tile_logic = 1
        _pinfo = self.get_tile_pinfo(tile_idx=tile_logic)

        seg_dict: Mapping[str, int] = self.params['seg_dict']
        w: int = self.params['w']
        ridx_tail: int = self.params['ridx_tail']
        ridx_gm: int = self.params['ridx_gm']
        export_tail: bool = self.params['export_tail']
        is_dum: bool = self.params['is_dum']

        # set number of columns
        seg_gm = seg_dict['gm']
        seg_tail = seg_dict['tail']
        if seg_tail % 2:
            raise ValueError(f'This generator requires seg_tail={seg_tail} to be even.')
        gm_row_info = _pinfo.get_row_place_info(ridx_gm).row_info
        if self.can_abut_mos(gm_row_info):
            seg_sep = 0
        else:
            seg_sep = 4
        seg_tot = max(seg_tail, 2 * seg_gm + seg_sep)
        seg_tot2 = seg_tot // 2
        seg_tail2 = seg_tail // 2
        seg_sep2 = seg_sep // 2

        # --- Placement --- #
        if seg_tail2 % 2 == 1:
            sup_term, share_term = MOSPortType.S, MOSPortType.D
            g_on_s = False
        else:
            sup_term, share_term = MOSPortType.D, MOSPortType.S
            g_on_s = True

        # tail
        tail_inst = self.add_mos(ridx_tail, seg_tot2 - seg_tail2, seg_tail, tile_idx=tile_logic, w=w, g_on_s=g_on_s)
        _row_info = _pinfo.get_row_place_info(ridx_tail).row_info
        w_tail, th_tail = _row_info.width, _row_info.threshold

        # gm cells
        gm_left_inst = self.add_mos(ridx_gm, seg_tot2 - seg_sep2, seg_gm, tile_idx=tile_logic, w=w, flip_lr=True)
        gm_right_inst = self.add_mos(ridx_gm, seg_tot2 + seg_sep2, seg_gm, tile_idx=tile_logic, w=w)
        w_gm, th_gm = gm_row_info.width, gm_row_info.threshold

        # tap cells
        tap_conn = self.add_substrate_contact(0, 0, tile_idx=tile_tap, seg=seg_tot, port_mode=SubPortMode.ODD)

        self.set_mos_size()

        # --- Routing --- #
        tr_manager = self.tr_manager
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xxm_layer = ym_layer + 1

        # 1. source terminals of tail transistor go to supplies
        tap_tid = self.get_track_id(0, MOSWireType.DS, 'sup', tile_idx=tile_tap)
        if self.can_extend_ds_conn(g_side=True, threshold=th_tail):
            tap_hm = self.connect_to_tracks([tail_inst[sup_term], tap_conn], tap_tid)
        else:
            raise NotImplementedError('Redo routing.')
        tap_xxm = self.connect_via_stack(tr_manager, tap_hm, xxm_layer, 'sup', alternate_o=True)
        self.add_pin('VSS', tap_xxm)

        # 2. source terminals of gm transistors connect to drain terminals of tail: connect on 2 hm_layer wires to
        # improve connection
        tail_tid0 = self.get_track_id(ridx_tail, MOSWireType.DS, 'sig_hs', tile_idx=tile_logic)
        tail_tid1 = self.get_track_id(ridx_gm, MOSWireType.DS, 'sig_hs', wire_idx=-1, tile_idx=tile_logic)
        tail1_vm = None
        if self.can_extend_ds_conn(g_side=False, threshold=th_gm):
            tail0 = self.connect_to_tracks([gm_left_inst.s, gm_right_inst.s, tail_inst[share_term]], tail_tid0,
                                           wire_lower=tail_inst[share_term].lower)
            tail1 = self.connect_to_tracks([gm_left_inst.s, gm_right_inst.s, tail_inst[share_term]], tail_tid1)
        else:
            tail0 = self.connect_to_tracks(tail_inst[share_term], tail_tid0, wire_lower=tail_inst[share_term].lower)
            tail1 = self.connect_to_tracks([gm_left_inst.s, gm_right_inst.s], tail_tid1)
            # connect on vm_layer
            tail1_vm = self.connect_via_stack(self.tr_manager, tail1, vm_layer, 'sig_hs',
                                              mlm_dict={vm_layer: MinLenMode.LOWER})
            self.connect_to_track_wires(tail0, tail1_vm)
        self.add_pin('v_tail', [tail0, tail1], hide=is_dum or not export_tail)

        # 3. gate terminals of tail transistor
        tail_g_tid = self.get_track_id(ridx_tail, MOSWireType.G, 'sig_hs', tile_idx=tile_logic)
        tail_g = self.connect_to_tracks(tail_inst.g, tail_g_tid)
        tail_g_xxm = self.connect_via_stack(tr_manager, tail_g, xxm_layer, 'sig_hs',
                                            mlm_dict={vm_layer: MinLenMode.LOWER, ym_layer: MinLenMode.LOWER})
        if is_dum:
            self.connect_wires([tail_inst.g, tail_inst[sup_term]])
        else:
            self.add_pin('v_tail_g', tail_g_xxm)

        # 4. get gate connections of gm transistors
        gate_tid = self.get_track_id(ridx_gm, MOSWireType.G, 'sig_hs', tile_idx=tile_logic)
        gate_left = self.connect_to_tracks(gm_left_inst.g, gate_tid)
        gate_right = self.connect_to_tracks(gm_right_inst.g, gate_tid)
        mlm_gate = {vm_layer: MinLenMode.UPPER, ym_layer: MinLenMode.UPPER}
        _, ym_locs_sig = tr_manager.place_wires(ym_layer, ['sig', 'sig', 'sig'], center_coord=seg_tot2 * self.sd_pitch)
        l_coord = self.grid.track_to_coord(ym_layer, ym_locs_sig[0])
        r_coord = self.grid.track_to_coord(ym_layer, ym_locs_sig[-1])
        gate_left_hm = self.add_wires(hm_layer, gate_tid.base_index, gate_left.lower,
                                      min(l_coord, gate_left.upper), width=gate_tid.width)
        gate_right_hm = self.add_wires(hm_layer, gate_tid.base_index, max(r_coord, gate_right.lower), gate_right.upper,
                                       width=gate_tid.width)
        gate_left_xxm = self.connect_via_stack(tr_manager, gate_left_hm, xxm_layer, 'sig', 0, -1, mlm_gate)
        gate_right_xxm = self.connect_via_stack(tr_manager, gate_right_hm, xxm_layer, 'sig', 0, 1, mlm_gate)
        self.add_pin('v_inp', gate_left_xxm)
        self.add_pin('v_inm', gate_right_xxm)

        # 5. get drain connections of gm transistors
        drain_tid = self.get_track_id(ridx_gm, MOSWireType.DS, 'sig_hs', wire_idx=0, tile_idx=tile_logic)
        drain_left = self.connect_to_tracks(gm_left_inst.d, drain_tid)
        drain_right = self.connect_to_tracks(gm_right_inst.d, drain_tid)
        mlm_drain = {vm_layer: MinLenMode.LOWER, ym_layer: MinLenMode.LOWER}
        _, ym_locs_sig_hs = self.tr_manager.place_wires(ym_layer, ['sig_hs', 'sig_hs', 'sig_hs'],
                                                        center_coord=seg_tot2 * self.sd_pitch)
        l_coord = self.grid.track_to_coord(ym_layer, ym_locs_sig_hs[0])
        r_coord = self.grid.track_to_coord(ym_layer, ym_locs_sig_hs[-1])
        drain_left_hm = self.add_wires(hm_layer, drain_tid.base_index, drain_left.lower,
                                       min(l_coord, drain_left.upper), width=drain_tid.width)
        drain_right_hm = self.add_wires(hm_layer, drain_tid.base_index, max(r_coord, drain_right.lower),
                                        drain_right.upper, width=drain_tid.width)
        if self.can_extend_ds_conn(g_side=False, threshold=th_gm):
            drain_left_xxm = self.connect_via_stack(tr_manager, drain_left_hm, xxm_layer, 'sig_hs', 0, -1, mlm_drain)
            drain_right_xxm = self.connect_via_stack(tr_manager, drain_right_hm, xxm_layer, 'sig_hs', 0, 1, mlm_drain)
        else:
            # don't collide with vm_layer for tail connection
            coords_l, coords_r = [], []
            for _widx in range((tail1_vm.track_id.num - 1) // 2):
                _tidx0 = self.tr_manager.get_next_track(vm_layer, tail1_vm[_widx].track_id.base_index,
                                                        'sig_hs', 'sig_hs', 1)
                coords_l.append(self.grid.track_to_coord(vm_layer, _tidx0))
                _tidx1 = self.tr_manager.get_next_track(vm_layer, tail1_vm[-1 - _widx].track_id.base_index,
                                                        'sig_hs', 'sig_hs', -1)
                coords_r.insert(0, self.grid.track_to_coord(vm_layer, _tidx1))
            drain_left_xm = self.connect_via_stack(tr_manager, drain_left_hm, xm_layer, 'sig_hs', 0, -1, mlm_drain,
                                                   coord_list_o_override=coords_l)
            drain_right_xm = self.connect_via_stack(tr_manager, drain_right_hm, xm_layer, 'sig_hs', 0, 1, mlm_drain,
                                                    coord_list_o_override=coords_r)
            drain_left_xxm = self.connect_via_stack(tr_manager, drain_left_xm, xxm_layer, 'sig_hs', 0, -1, mlm_drain)
            drain_right_xxm = self.connect_via_stack(tr_manager, drain_right_xm, xxm_layer, 'sig_hs', 0, 1, mlm_drain)
        self.add_pin('i_outm', drain_left_xxm, hide=is_dum)
        self.add_pin('i_outp', drain_right_xxm, hide=is_dum)

        # get schematic parameters
        self.sch_params = dict(
            lch=_pinfo.lch,
            w_dict=dict(tail=w_tail, gm=w_gm),
            th_dict=dict(tail=th_tail, gm=th_gm),
            seg_dict=seg_dict,
            export_tail=export_tail,
            is_dum=is_dum,
        )


class GmStageGR(GuardRing):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        GuardRing.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        ans = dict(**GmStage.get_params_info())
        ans.update(
            pmos_gr='pmos guard ring tile name.',
            nmos_gr='nmos guard ring tile name.',
            edge_ncol='Number of columns on guard ring edge.  Use 0 for default.',
        )
        return ans

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        ans = dict(**GmStage.get_default_param_values())
        ans.update(
            pmos_gr='pgr',
            nmos_gr='ngr',
            edge_ncol=0,
        )
        return ans

    def get_layout_basename(self) -> str:
        return self.__class__.__name__

    def draw_layout(self) -> None:
        params = self.params
        pmos_gr: str = params['pmos_gr']
        nmos_gr: str = params['nmos_gr']
        edge_ncol: int = params['edge_ncol']

        core_params = params.copy(remove=['pmos_gr', 'nmos_gr', 'edge_ncol'])
        master = self.new_template(GmStage, params=core_params)

        sep_ncol_left = sep_ncol_right = master.gr_sub_sep_col
        sep_ncol = (sep_ncol_left, sep_ncol_right)

        inst, sup_list = self.draw_guard_ring(master, pmos_gr, nmos_gr, sep_ncol, edge_ncol, export_pins=False)

        # --- Routing --- #
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xxm_layer = ym_layer + 1
        yym_layer = xxm_layer + 1
        x3m_layer = yym_layer + 1

        # guard ring supply connections
        vdd_hm_list = []
        for (_, vdd_list) in sup_list:
            if vdd_list:
                vdd_hm_list.extend(vdd_list)
        vdd_dict0, vdd_dict1 = {}, {}
        mlm_dict0 = {vm_layer: MinLenMode.LOWER, ym_layer: MinLenMode.LOWER}
        mlm_dict1 = {vm_layer: MinLenMode.UPPER, ym_layer: MinLenMode.UPPER}
        vdd_xxm0 = self.connect_via_stack(self.tr_manager, vdd_hm_list[0], xxm_layer, 'guard', mlm_dict=mlm_dict0,
                                          ret_warr_dict=vdd_dict0)
        vdd_xxm1 = self.connect_via_stack(self.tr_manager, vdd_hm_list[-1], xxm_layer, 'guard', mlm_dict=mlm_dict1,
                                          ret_warr_dict=vdd_dict1)

        for _layer in (vm_layer, ym_layer):
            self.connect_wires([vdd_dict0[_layer][0], vdd_dict0[_layer][-1],
                                vdd_dict1[_layer][0], vdd_dict1[_layer][-1]])

        # --- bring all signals to x3m_layer without colliding on yym_layer --- #
        # get all yym_layer tracks so that alternate tracks can be used to go from xxm_layer to x3m_layer
        tidx_l = self.grid.coord_to_track(yym_layer, vdd_xxm0.lower, RoundMode.GREATER_EQ)
        tidx_r = self.grid.coord_to_track(yym_layer, vdd_xxm0.upper, RoundMode.LESS_EQ)
        num_yym = self.tr_manager.get_num_wires_between(yym_layer, 'sig_hs', tidx_l, 'sig_hs', tidx_r, 'sig_hs') + 2
        if num_yym & 1 == 0:
            num_yym -= 1
        yym_tidx_list = self.tr_manager.spread_wires(yym_layer, ['sig_hs'] * num_yym, tidx_l, tidx_r,
                                                     ('sig_hs', 'sig_hs'))
        yym_coords = [self.grid.track_to_coord(yym_layer, _tidx) for _tidx in yym_tidx_list]

        # VDD on x3m_layer
        vdd_x3m0 = self.connect_via_stack(self.tr_manager, vdd_xxm0, x3m_layer, 'guard',
                                          mlm_dict={yym_layer: MinLenMode.LOWER}, coord_list_o_override=yym_coords[::2])
        vdd_x3m1 = self.connect_via_stack(self.tr_manager, vdd_xxm1, x3m_layer, 'guard',
                                          mlm_dict={yym_layer: MinLenMode.UPPER}, coord_list_o_override=yym_coords[::2])
        self.add_pin('VDD', self.connect_wires([vdd_x3m0, vdd_x3m1])[0])

        # x3m_layers between two VDD tracks
        x3m_tidx_list = self.tr_manager.spread_wires(x3m_layer, ['sig_hs', 'sig_hs', 'sig_hs', 'sig_hs', 'sig_hs'],
                                                     vdd_x3m0.track_id.base_index, vdd_x3m1.track_id.base_index,
                                                     ('sig_hs', 'sig_hs'))
        x3m_coords = [self.grid.track_to_coord(x3m_layer, _tidx) for _tidx in x3m_tidx_list]

        # VSS on x3m_layer
        vss_x3m = self.connect_via_stack(self.tr_manager, inst.get_pin('VSS'), x3m_layer, 'sup',
                                         coord_list_o_override=yym_coords[1::2], coord_list_p_override=[x3m_coords[1]])
        self.add_pin('VSS', vss_x3m)

        # left input and output on x3m_layer
        v_inp = inst.get_pin('v_inp')
        i_outm = inst.get_pin('i_outm')
        _l0 = max(v_inp.lower, i_outm.lower)
        _r0 = min(v_inp.upper, i_outm.upper)
        _coords0 = get_sub_list(yym_coords, _l0, _r0)

        v_inp = self.connect_via_stack(self.tr_manager, v_inp, x3m_layer, 'sig_hs',
                                       mlm_dict={yym_layer: MinLenMode.UPPER},
                                       coord_list_o_override=_coords0[1::2], coord_list_p_override=[x3m_coords[-2]])
        self.add_pin('v_inp', v_inp)
        i_outm = self.connect_via_stack(self.tr_manager, i_outm, x3m_layer, 'sig_hs',
                                        mlm_dict={yym_layer: MinLenMode.LOWER},
                                        coord_list_o_override=_coords0[::2], coord_list_p_override=[x3m_coords[-3]])
        self.add_pin('i_outm', i_outm)

        # right input and output on x3m_layer
        v_inm = inst.get_pin('v_inm')
        i_outp = inst.get_pin('i_outp')
        _l1 = max(v_inm.lower, i_outp.lower)
        _r1 = min(v_inm.upper, i_outp.upper)
        _coords1 = get_sub_list(yym_coords, _l1, _r1)

        v_inm = self.connect_via_stack(self.tr_manager, v_inm, x3m_layer, 'sig_hs',
                                       mlm_dict={yym_layer: MinLenMode.UPPER},
                                       coord_list_o_override=_coords1[-2::-2][::-1],
                                       coord_list_p_override=[x3m_coords[-2]])
        self.add_pin('v_inm', v_inm)
        i_outp = self.connect_via_stack(self.tr_manager, i_outp, x3m_layer, 'sig_hs',
                                        mlm_dict={yym_layer: MinLenMode.LOWER},
                                        coord_list_o_override=_coords1[::-2][::-1],
                                        coord_list_p_override=[x3m_coords[-3]])
        self.add_pin('i_outp', i_outp)

        # v_tail_g is DC and exported on xxm_layer
        self.reexport(inst.get_port('v_tail_g'))

        # v_tail is exported for debugging
        self.reexport(inst.get_port('v_tail'))

        # update schematic parameters
        self.sch_params = dict(
            **self.sch_params,
            guard_ring=True,
        )


def get_sub_list(orig_list: Sequence[int], low: int, high: int) -> Sequence[int]:
    new_list = []
    for ele in orig_list:
        if ele >= low:
            if ele > high:
                break
            new_list.append(ele)
    return new_list
