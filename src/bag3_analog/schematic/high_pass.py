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

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param
from bag.math import float_to_si_string


# noinspection PyPep8Naming
class bag3_analog__high_pass(Module):
    """Module for library bag3_analog cell high_pass.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'high_pass.yaml')))

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
            l='unit resistor length, in resolution units',
            w='unit resistor width, in resolution units',
            intent='resistor type.',
            nser='number of resistors in series in a branch.',
            ndum='number of dummy resistors.',
            res_in_info='input metal resistor information.',
            res_out_info='output metal resistor information.',
            sub_name='substrate name. Empty string to disable.',
            is_differential='is_differential',
            cap_val='Schematic value for analogLib cap',
            extracted='True if doing LVS or extraction. Removes analogLib caps'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            sub_name='VDD',
            is_differential=True,
            cap_val=1e-9,
            extracted=False,
        )

    def design(self, l: int, w: int, intent: str, nser: int, ndum: int,
               res_in_info: Dict, res_out_info: Dict, sub_name: str,
               is_differential: bool, cap_val: float, extracted: bool) -> None:
        """"""
        if not is_differential:
            # TODO: add differential option
            raise RuntimeError("Currently only supporting differential HPF")
        name_info = (('XCAPP', 'XRESP', 'inp', 'outp', 'biasp'),
                     ('XCAPN', 'XRESN', 'inn', 'outn', 'biasn'))

        # Design resistors
        for info_tuple in name_info:
            res_name = info_tuple[1]
            out_name = info_tuple[-2]
            self.instances[res_name].design(l=l, w=w, intent=intent)
            term_list = [(res_name + f'{x}',
                         [('PLUS', out_name if x == 0 else f'{out_name}{x-1}'),
                          ('MINUS', info_tuple[-1] if x == nser - 1 else f'{out_name}{x}'), ('BULK', sub_name)])
                         for x in range(nser)]
            self.array_instance(res_name, inst_term_list=term_list, dx=100, dy=0)

        # Design dummy resistors
        if not ndum:
            self.remove_instance('XRESD')
        else:
            self.instances['XRESD'].design(l=l, w=w, intent=intent)
            term_list = [('XRESD' + f'{x}',
                          [('PLUS', sub_name), ('MINUS', sub_name), ('BULK', sub_name)])
                         for x in range(ndum)]
            self.array_instance('XRESD', inst_term_list=term_list, dx=100, dy=0)

        # Design capacitors
        if extracted:
            self.remove_instance(name_info[0][0])
            self.remove_instance(name_info[1][0])
        else:
            cap_val_str = float_to_si_string(cap_val)
            self.instances[name_info[0][0]].set_param('c', cap_val_str)
            self.instances[name_info[1][0]].set_param('c', cap_val_str)

        # Design metal resistors
        self.instances['XMRESP1'].design(layer=res_in_info[0], w=res_in_info[1], l=res_in_info[2])
        self.instances['XMRESN1'].design(layer=res_in_info[0], w=res_in_info[1], l=res_in_info[2])
        self.instances['XMRESP2'].design(layer=res_out_info[0], w=res_out_info[1], l=res_out_info[2])
        self.instances['XMRESN2'].design(layer=res_out_info[0], w=res_out_info[1], l=res_out_info[2])

        self.rename_pin('BULK', sub_name)
