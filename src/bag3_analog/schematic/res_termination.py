# SPDX-License-Identifier: Apache-2.0
# Copyright 2020 Blue Cheetah Analog Design Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# -*- coding: utf-8 -*-

from typing import Dict, Any

import pkg_resources
from pathlib import Path

from pybag.enum import TermType

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag3_analog__res_termination(Module):
    """Module for library bag3_analog cell res_termination.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'res_termination.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Dict[str, str]:
        """Returns a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : Optional[Dict[str, str]]
            dictionary from parameter names to descriptions.
        """
        return dict(
            w="Resistor width",
            l="Resistor length",
            res_type="Resistor flavor",
            nx="Number of units in the X direction",
            ny="Number of units in the Y direction",
            nx_dum="Number of dummies on each side, X direction",
            ny_dum="Number of dummies on each side, Y direction",
            export_mid="True to export mid, so this can be used differentially",
            sup_name="Supply name to connect to bulk or float"
        )

    def design(self, w: int, l: int, res_type: str, nx: int, ny, nx_dum: int, ny_dum: int, export_mid: bool,
               sup_name: str) -> None:
        unit_params = dict(w=w, l=l, intent=res_type)
        npar = nx - 2 * nx_dum
        nser = ny - 2 * ny_dum
        ndum = nx * ny - npar * nser

        rp_name = 'XRES_P'
        inst_name_list = [rp_name]

        rm_name = 'XRES_M'
        if export_mid:
            inst_name_list.append(rm_name)
            nser = nser // 2

        dum_name = 'XRES_DUM'
        if ndum > 0:
            inst_name_list.append(dum_name)

        self.array_instance('XRES', inst_name_list, dy=-200)

        # design dummies
        if ndum > 0:
            self.design_resistor(dum_name, unit_params, npar=ndum, plus=sup_name, minus=sup_name, bulk=sup_name)

        # design resistors
        if export_mid:
            self.design_resistor(rp_name, unit_params, nser, npar, 'PLUS', 'MID', 'p', sup_name, True)
            self.design_resistor(rm_name, unit_params, nser, npar, 'MID', 'MINUS', 'm', sup_name, True)
            self.add_pin('MID', TermType.inout)
        else:
            self.design_resistor(rp_name, unit_params, nser, npar, 'PLUS', 'MINUS', 'p', sup_name, True)

        # Change bulk pin name
        self.rename_pin('BULK', sup_name)
