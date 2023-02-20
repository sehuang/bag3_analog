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

from typing import Mapping, Any, Union

import pkg_resources
from pathlib import Path
from enum import IntEnum

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param

from pybag.enum import TermType


class LayMode(IntEnum):
    TOP = 0     # for top level DRC and LVS
    EXT = 1    # for extraction
    EM = 2   # for em simulation


# noinspection PyPep8Naming
class bag3_analog__drv_shunt_peak(Module):
    """Module for library bag3_analog cell drv_shunt_peak.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'drv_shunt_peak.yaml')))

    def __init__(self, database: ModuleDB, params: Param, **kwargs: Any) -> None:
        Module.__init__(self, self.yaml_file, database, params, **kwargs)

    @classmethod
    def get_params_info(cls) -> Mapping[str, str]:
        """Returns a dictionary from parameter names to descriptions.

        Returns
        -------
        param_info : Optional[Mapping[str, str]]
            dictionary from parameter names to descriptions.
        """
        return dict(
            gm='Parameters for gm cells',
            res='Parameters for differential resistors',
            ind='Parameters for shunt differential inductor',
            lay_mode='Layout mode, TOP by default.',
        )

    def design(self, gm: Mapping[str, Any], res: Mapping[str, Any], ind: Mapping[str, Any],
               lay_mode: Union[LayMode, str]) -> None:
        """To be overridden by subclasses to design this module.

        This method should fill in values for all parameters in
        self.parameters.  To design instances of this module, you can
        call their design() method or any other ways you coded.

        To modify schematic structure, call:

        rename_pin()
        delete_instance()
        replace_instance_master()
        reconnect_instance_terminal()
        restore_instance()
        array_instance()
        """
        remove_pins = []

        # gm cell
        gen_gm = lay_mode is LayMode.TOP or lay_mode is LayMode.EXT
        if gen_gm:
            self.instances['XGM'].design(**gm)
        else:
            self.remove_instance('XGM')
            remove_pins.extend(['v_inp', 'v_inm', 'v_tail_g', 'v_tail', 'i_outp', 'i_outm', 'VSS'])

        # resistors
        gen_res = lay_mode is LayMode.TOP or lay_mode is LayMode.EXT
        if gen_res:
            self.instances['XRES'].design(**res)
        else:
            self.remove_instance('XRES')
            remove_pins.extend(['VDD'])

        # shunt inductors
        gen_ind = lay_mode is LayMode.TOP or lay_mode is LayMode.EM
        if gen_ind:
            self.instances['XIND'].design(**ind)
            if lay_mode is LayMode.TOP:
                self.reconnect_instance_terminal('XIND', 'P2_R', 'VDD')
            else:
                # lay_mode is LayMode.EM
                self.reconnect_instance('XIND', [('P1', 'P1'), ('P3', 'P3'), ('P2', 'P2'),
                                                 ('P13_R', 'P13_R'), ('P2_R', 'P2_R')])
                self.rename_pin('ind_p', 'P1')
                self.rename_pin('ind_m', 'P3')
                for pin in ('P2', 'P13_R', 'P2_R'):
                    self.add_pin(pin, TermType.inout)
        else:
            self.remove_instance('XIND')

        for pin in remove_pins:
            self.remove_pin(pin)
