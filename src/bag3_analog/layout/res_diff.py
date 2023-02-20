"""This module defines differential resistors."""

from typing import Mapping, Any, Optional, Type, cast

from bag.util.immutable import Param
from bag.design.module import Module
from bag.layout.template import TemplateDB
from bag.layout.routing.base import TrackManager, WireArray

from xbase.layout.res.base import ResBasePlaceInfo, ResArrayBase

from ..schematic.res_diff import bag3_analog__res_diff


class ResDiff(ResArrayBase):
    def __init__(self, temp_db: TemplateDB, params: Param, **kwargs: Any) -> None:
        ResArrayBase.__init__(self, temp_db, params, **kwargs)

    @classmethod
    def get_schematic_class(cls) -> Optional[Type[Module]]:
        return bag3_analog__res_diff

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
        npar_tot = nx - 2 * nx_dum
        nser = ny - 2 * ny_dum
        assert npar_tot & 1 == 0, f'nx - 2 * nx_dum = {npar_tot} should be even.'
        num_dum = nx * ny - npar_tot * nser

        # --- Routing of unit resistors --- #
        hm_layer = self.conn_layer + 1
        vm_layer = hm_layer + 1
        xm_layer = vm_layer + 1
        ym_layer = xm_layer + 1
        xxm_layer = ym_layer + 1

        # make 2 resistors out of all the resistor units
        rp_bot, rp_top = self.connect_units(warrs, nx_dum, nx // 2, ny_dum, ny - ny_dum)
        rm_bot, rm_top = self.connect_units(warrs, nx // 2, nx - nx_dum, ny_dum, ny - ny_dum)

        # Supply connections on xxm_layer
        sub_type = cast(ResBasePlaceInfo, self.place_info).res_config['sub_type_default']
        sup_top0 = self.connect_via_stack(self.tr_manager, bulk_warrs[vm_layer][0], xxm_layer, 'sup')
        sup_top1 = self.connect_via_stack(self.tr_manager, bulk_warrs[vm_layer][1], xxm_layer, 'sup')
        sup_name = 'VDD' if sub_type == 'ntap' else 'VSS'
        self.add_pin(sup_name, self.connect_wires([sup_top0, sup_top1])[0])

        # connect XRP and XRM from vm_layer to xxm_layer
        for _bot, _top, _suf in [(rp_bot, rp_top, 'p'), (rm_bot, rm_top, 'm')]:
            _in = self.connect_stack(self.tr_manager, _bot, xxm_layer, 'sig')
            _out = self.connect_stack(self.tr_manager, _top, xxm_layer, 'sig')
            self.add_pin(f'{_suf}_in', _in)
            self.add_pin(f'{_suf}_out', _out)

        self.sch_params = dict(
            load_params=dict(
                unit_params=unit_params,
                nser=nser,
                npar=npar_tot // 2,
            ),
            dum_params=dict(
                unit_params=unit_params,
                nser=1,
                npar=num_dum,
            ),
            sub_type=sub_type,
        )

    def connect_stack(self, tr_manager: TrackManager, warr: WireArray, top_layer: int, w_type: str = 'sig'):
        # this is different from connect_via_stack() as it does not connect the intermediate wires to reduce cap
        top_warr = []
        for _warr in warr.warr_iter():
            top_warr.append(self.connect_via_stack(tr_manager, _warr, top_layer, w_type))
        return self.connect_wires(top_warr)[0]
