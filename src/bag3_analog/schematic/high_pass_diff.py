# BSD 3-Clause License
#
# Copyright (c) 2018, Regents of the University of California
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

# -*- coding: utf-8 -*-

from typing import Dict, Any, Tuple, Union

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param
from bag.math import float_to_si_string


# noinspection PyPep8Naming
class bag3_analog__high_pass_diff(Module):
    """ Differential HPF
    # TODO: can we merge this with single HPF? or instantiate 2 subcells?
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'high_pass_diff.yaml')))

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
            ndum='number of dummy resistors (parallel, series).',
            res_in_info='input metal resistor information.',
            res_out_info='output metal resistor information.',
            sub_name='substrate name. Empty string to disable.',
            cap_val='Schematic value for analogLib cap',
            extracted='True if doing LVS or extraction. Removes analogLib caps'
        )

    @classmethod
    def get_default_param_values(cls) -> Dict[str, Any]:
        return dict(
            sub_name='VDD',
            cap_val=1e-9,
            extracted=False,
        )

    def design(self, l: int, w: int, intent: str, nser: int, ndum: Union[int, Tuple[int, int]],
               res_in_info: Tuple[int, int, int], res_out_info: Tuple[int, int, int], sub_name: str,
               cap_val: float, extracted: bool) -> None:
        """"""
        name_info = (('XCAPP', 'XRESP', 'inp', 'midp', 'biasp'),
                     ('XCAPN', 'XRESN', 'inn', 'midn', 'biasn'))

        # Design resistors
        unit_params = dict(l=l, w=w, intent=intent)
        for info_tuple in name_info:
            res_name = info_tuple[1]
            out_name = info_tuple[-2]
            self.design_resistor(res_name, unit_params, nser, 1,
                                 out_name, info_tuple[-1], f'{out_name}_x', sub_name)

        # Design dummy resistors
        dummy_info = [('XRESPD', 'biasp'), ('XRESND', 'biasn')]
        if isinstance(ndum, int):
            ndum = (ndum, 1)
        ndum_par, ndum_ser = ndum
        for name, bias in dummy_info:
            if not ndum:
                self.remove_instance(name)
            else:
                self.design_resistor(name, unit_params, ndum_ser, ndum_par, bias, bias, f'{bias}_dum_x', bulk=sub_name)

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
