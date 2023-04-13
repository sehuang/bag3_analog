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

from typing import Mapping, Any

import pkg_resources
from pathlib import Path

from bag.design.module import Module
from bag.design.database import ModuleDB
from bag.util.immutable import Param


# noinspection PyPep8Naming
class bag3_analog__rdac_decoder_row_col(Module):
    """Module for library bag3_analog cell rdac_decoder_row_col.

    Fill in high level description here.
    """

    yaml_file = pkg_resources.resource_filename(__name__,
                                                str(Path('netlist_info',
                                                         'rdac_decoder_row_col.yaml')))

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
            inv_params='Inverter parameters',
            and_params='Parameters for and_complex',
            pg_params='Parameters for pass gate',
            num_sel='Number of select inputs',
            decoder_type='"row" or "column"',
            num_rows='Number of passgate rows, used for "column" type',
        )

    @classmethod
    def get_default_param_values(cls) -> Mapping[str, Any]:
        return dict(decoder_type='row', num_rows=1)

    def get_master_basename(self) -> str:
        decoder_type: str = self.params['decoder_type']
        if decoder_type not in ['row', 'column']:
            raise ValueError(f'Unknown decoder_type={decoder_type}. Use "row" or "column".')
        return f'rdac_decoder_{decoder_type}'

    def design(self, inv_params: Mapping[str, Any], and_params: Mapping[str, Any], pg_params: Mapping[str, Any],
               num_sel: int, decoder_type: str, num_rows: int) -> None:
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
        dy = 250  # magic number

        # inverters
        sel_suf = f'<{num_sel - 1}:0>'
        self.instances['XINV'].design(**inv_params)
        self.rename_instance('XINV', f'XINV{sel_suf}', [('in', f'sel{sel_suf}'), ('out', f'selb{sel_suf}')])
        self.rename_pin('sel', f'sel{sel_suf}')

        # complex and gates
        num_and = 1 << num_sel
        and_config = []
        for idx in range(num_and):
            bin_idx = f'{idx:0b}'.zfill(num_sel)
            in_list = []
            for char_idx, bin_char in enumerate(bin_idx):
                sel_idx = num_sel - 1 - char_idx
                in_list.append(f'sel<{sel_idx}>' if bin_char == '1' else f'selb<{sel_idx}>')
            term_list = [('out', f'en<{idx}>'), ('outb', f'enb<{idx}>'), (f'in{sel_suf}', ','.join(in_list))]
            and_config.append((f'XAND{idx}', term_list))
        self.instances['XAND'].design(**and_params)
        self.array_instance('XAND', inst_term_list=and_config, dx=0, dy=dy)

        # passgates and current summers
        self.instances['XPG'].design(**pg_params)
        num_in = num_and * num_rows
        in_suf = f'<{num_in - 1}:0>'
        self.rename_pin('in', f'in{in_suf}')

        if decoder_type == 'row':
            # passgates
            self.rename_instance('XPG', f'XPG{in_suf}', [('s', f'in{in_suf}'), ('d', f'pg_out{in_suf}'),
                                                         ('en', f'en{in_suf}'), ('enb', f'enb{in_suf}')])

            # current summers
            self.instances['XCS'].design(nin=num_in)
            self.reconnect_instance_terminal('XCS', f'in{in_suf}', f'pg_out{in_suf}')
        else:  # decoder_type is 'column'
            self.instances['XCS'].design(nin=num_and)
            pg_config = []
            cs_config = []
            and_suf = f'<{num_and - 1}:0>'
            en_net = f'en{and_suf}'
            enb_net = f'enb{and_suf}'
            for idx in range(num_rows):
                _suf = f'<{(idx + 1) * num_and - 1}:{idx * num_and}>'
                pg_config.append((f'XPG{idx}{and_suf}', [('en', en_net), ('enb', enb_net), ('s', f'in{_suf}'),
                                                         ('d', f'pg_out{_suf}')]))
                cs_config.append((f'XCS{idx}', [(f'in{and_suf}', f'pg_out{_suf}'), ('out', f'out<{idx}>')]))
            self.rename_pin('out', f'out<{num_rows - 1}:0>')
            self.array_instance('XPG', inst_term_list=pg_config, dx=0, dy=dy)
            self.array_instance('XCS', inst_term_list=cs_config, dx=0, dy=dy)
