from ScopeFoundry import HardwareComponent
from ScopeFoundryHW.quantum_composer.quantum_composer_9530 import QuantumComposer9530, spulse_states_dict, pulse_states_dict

class QuantumComposerHW(HardwareComponent):
    
    name = 'quantum_composer'
    
    
    def setup(self):
        
        
        self.settings.New('port', dtype=str, initial='COM11')
        
        #"T0"
        for name, state in spulse_states_dict.items():
            print( name, state )
            self.settings.New("T0_"+name, dtype=state.dtype, **state.kwargs)
   
        for cname in ["CHA", "CHB", "CHC", "CHD"]:
            for name, state in pulse_states_dict.items():
                self.settings.New(cname+"_"+name, dtype=state.dtype, **state.kwargs)


    def connect(self):
        self.qc = QuantumComposer9530(port=self.settings['port'], debug=self.settings['debug_mode'])
        
        for name in spulse_states_dict.keys():
            lq = self.settings.get_lq("T0_"+name)
            lq.connect_to_hardware(
                read_func = lambda x=name: self.qc.read_spulse_state(x),
                write_func = lambda val, x=name: self.qc.write_spulse_state(x, val)
                )
        
        for cname in ["CHA", "CHB", "CHC", "CHD"]:
            for name in pulse_states_dict.keys():
                lq = self.settings.get_lq(cname+"_"+name)
                lq.connect_to_hardware(
                    read_func = lambda cname=cname, name=name: self.qc.read_pulse_state(cname, name),
                    write_func = lambda val, cname=cname, name=name: self.qc.write_pulse_state(cname, name, val)
                    )
                
        self.read_from_hardware()
                

    def disconnect(self):
        self.settings.disconnect_all_from_hardware()
        if hasattr(self, 'qc'):
            self.qc.close()
            del self.qc

if __name__ == '__main__':
    from ScopeFoundry import BaseMicroscopeApp
    import sys
    
    app = BaseMicroscopeApp([])
    app.add_hardware(QuantumComposerHW(app))
    app.hardware['quantum_composer'].settings['debug_mode'] = True    
    app.hardware['quantum_composer'].settings['connected'] = True
    sys.exit(app.exec_())