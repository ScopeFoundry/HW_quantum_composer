import serial
from collections import namedtuple, OrderedDict

class QCChannel(object):
    def __init__(self,cnum, name):
        self.cnum = cnum
        self.name = name
        
    def __str__(self):
        return "QC Chan: {} {}".format(self.cnum, self.name)
    
    def __repr__(self):
        return "QCChannel({},{})".format(self.cnum,self.name)
    
#QCChannel = namedtuple('QCChannel', ['cnum', 'name'])

def str2bool(v):
    print( "str2bool", repr(v.lower()))
    ret = v.lower() in ("1", "on", "yes", "true", "t")
    print ("-->", ret)
    return ret

system_states = [
    ('state', str, "SYSTem:STATe"),

]

"""    SYSTem              Subsystem
        STATe               Query

        BEEPer              Subsystem
            STATe               Boolean
            VOLume              Integer

        COMMunicate         Subsystem

            SERial              Subsystem
                BAUD                List
                USB                 List
                ECHo                Boolean
        AUTorun             Boolean
        KLOCk               Boolean
        CAPS                Boolean
        VERSion             Query
        SERNumber           Query
        INFOrmation         Query
        NSID                Query
"""


class QCState(object):
    def __init__(self, name, dtype, cmd, **kwargs):
        self.name = name
        self.dtype = dtype
        self.cmd = cmd
        self.kwargs = kwargs
        for k,v in kwargs.items():
            setattr(self, k, v)
        
S = QCState
spulse_states = [
    S('counter_state', bool, ":SPULSE:COUNter:STATe"),
    #S('counter_counts', int, ":SPULSE:COUNter:COUNt", ro=True),
    S('state', bool, ":SPULSE:STATe"),
    S('period', float, ":SPULSE:PERiod", unit='s', vmin=50e-9, vmax=5000, si=True),
    S('mode', str, ":SPULSE:MODE", choices=("NORM", "SING", "BURS", "DCYC")),
    S('burst_counter', int, ":PULSE0:BCOunter", vmin=1, vmax=9999999),
    S('duty_on_counter', int, ":PULSE0:PCOunter", vmin=1, vmax=9999999),
    S('duty_off_counter', int, ":PULSE0:OCOunter", vmin=1, vmax=9999999),

    S('input_clock_mode', str, ":SPULSE:ICLock:MODE", choices=("INT", "EXT", "XPL")),
    S('input_clock_rate', int, ":SPULSE:ICLock:RATE", unit='MHz', vmin=10, vmax=100),
    S('input_clock_level', float, ":SPULSE:ICLock:LEVel", vmin=0.02, vmax=2.5, unit='V'),
    S('input_clock_lock_lost_option', str, ":SPULSE:ICLock:OPTion", choices=("FORCE", "LAST")),

    S('output_clock', str, ":SPULSE:OCLock"),
    
    S('gate_mode', str, ":SPULSE:GATE:MODE", choices=("DIS", "PULS", "OUTP", "CHAN")),
    S('gate_active_logic', str, ":SPULSE:GATE:LOGic", choices=("LOW", "HIGH")),
    S('gate_edge', str, ":SPULSE:GATE:EDGE", choices=("RIS", "FALL")),
    S('gate_level', float, ":SPULSE:TRIG:LEVel", vmin=0.20, vmax=15.0),
    
    S('trigger_mode', str, ":SPULSE:TRIG:MODE", choices=("DIS", "TRIG", "DUAL")),
    S('trigger_edge', str, ":SPULSE:TRIG:EDGE", choices=("RIS", "FALL")),
    S('trigger_level', str, ":SPULSE:TRIG:LEVel", vmin=0.20, vmax=15.0 )
]
spulse_states_dict = OrderedDict( 
    [ (state.name , state) 
            for state in spulse_states] )
        
    
pulse_states = [
    S('state', bool, "STATE"),
    S('width', float, "WIDTH", si=True, unit='s'),
    S('delay', float, "DELAY", si=True, unit='s'),
    S('sync_to', str, "SYNC"),
    S('mux', int, "MUX"),
    S('polarity', str, "POLarity"),
    S('output_mode', str, "OUTP:MODE"),
    S('output_amplitude', float, "OUTP:AMPL"),
    S('mode', str, "CMODE"),
    S('burst_counter', float, "BCOunter"),
    S('duty_on_counter', int, "PCOunter"),
    S('duty_off_counter', int, "OCOunter"),
    S('wait_counter', int, "WCOunter"),
    S('trigger_source', str, "CTRIG"),
    S('gate_mode', str, "CGATe"),
    S('gate_active_logic', str, "CLOGic")
]

pulse_states_dict = OrderedDict( 
    [ (state.name, state) 
            for state in pulse_states] )

error_codes = {
    1: "Incorrect prefix to start command",
    2: "Missing command keyword",
    3: "Invalid command keyword",
    4: "Missing parameter",
    5: "Invalid parameter",
    6: "Query only, command needs a question mark",
    7: "Invalid query, command does not have a query form",
    8: "Command unavailable in current system state"
    }

class QuantumComposer9530(object):

    def __init__(self, port="COM1", debug=False):
        
        "echo should be disabled"
        
        self.debug = debug
        self.port = port
        
        self.ser = serial.Serial(port=port, baudrate = 115200, bytesize=8, parity='N', 
                stopbits=1, xonxoff=0, rtscts=0, timeout=0.1)

        self._ask(":SYST:COMM:SER:ECHO OFF")
        self.ser.readline() # incase ECHO was enabled, clears the serial buffer of the echo

        resp = self._ask(":INST:CATalog?") #"T0, CHA, CHB, CHC, CHD"
        resp = resp.split(',')
        
        self.channel_names = [x.strip() for x in resp]
        self.chan_name_dict = dict( [(name, i) for i, name in enumerate(self.channel_names)] )
        self.num_channels = len(self.channel_names)
        
        self.chan = [None,]*self.num_channels
        
        for ii, cname in enumerate(self.channel_names):
            self.chan[ii] = QCChannel(ii, cname)
        for ii in range(1, self.num_channels):
            self.get_pulse(ii)

        self.get_spulse()
        
    def close(self):
        self.ser.close()
        
    def _ask(self, cmd):
        
        if self.debug: print("cmd: ", repr(cmd))
        self.ser.write(cmd.encode('ascii') + b"\r\n")
        
        resp = self.ser.readline()
        if self.debug: print("resp:", repr(resp))
        return resp.strip().decode('ascii')

    def get_idn(self):
        resp = self._ask("*IDN?")
        
        "QC,9534,02311,1.2.2-1.2.0"
        
        resp = resp.split(b',')
        assert resp[0] == b'QC'
        self.model = resp[1]
        self.serial_number = resp[2]
        self.version = resp[3]
        return resp

        
    def get_spulse(self):
    
        self.spulse = self.chan[0]
        chan = 0

        #:SPULSE:COUNter: TODO
        
        self.spulse.state = bool(self._ask(":SPULSE:STATe?"))
        self.spulse.period = float(self._ask(":SPULSE:PERiod?"))
        self.spulse.mode = self._ask(":SPULSE:MODE?")

        self.spulse.burst_counter = int(self._ask(":PULSE0:BCOunter?" ))
        self.spulse.duty_on_counter = int(self._ask(":PULSE0:PCOunter?" ))
        self.spulse.duty_off_counter = int(self._ask(":PULSE0:OCOunter?" ))

        self.spulse.input_clock_mode = self._ask(":SPULSE:ICLock:MODE?")
        self.spulse.input_clock_rate = int(self._ask(":SPULSE:ICLock:RATE?"))
        self.spulse.input_clock_level = float(self._ask(":SPULSE:ICLock:LEVel?"))
        self.spulse.input_clock_lock_lost_option = self._ask(":SPULSE:ICLock:OPTion?")

        self.spulse.output_clock = self._ask(":SPULSE:OCLock?")

        self.spulse.gate_mode = self._ask(":SPULSE:GATE:MODE?")
        self.spulse.gate_active_logic = self._ask(":SPULSE:GATE:LOGic?")
        self.spulse.gate_edge = self._ask(":SPULSE:GATE:EDGE?")
        self.spulse.gate_level = float(self._ask(":SPULSE:GATE:LEVel?"))

        self.spulse.trigger_mode = self._ask(":SPULSE:TRIG:MODE?")
        self.spulse.trigger_edge = self._ask(":SPULSE:TRIG:EDGE?")
        self.spulse.trigger_level = float(self._ask(":SPULSE:TRIG:LEVel?"))
        
        return self.spulse
    
    def read_spulse_state(self, x):
        if self.debug:
            print("read_spulse_state", x)
        state = spulse_states_dict[x]
        resp = self._ask(state.cmd+"?")
        if resp[0] == '?':
            raise IOError("QC error: " + error_codes.get(int(resp[1]), resp))
        if state.dtype == str:
            resp = resp #.decode('ascii')
        elif state.dtype == bool:
            str2bool(resp.strip())
        return state.dtype(resp)
    
    def write_spulse_state(self, name, val):
        if self.debug:
            print("write_spulse_state", name, val)
        state = spulse_states_dict[name]
        
        if state.dtype == bool:
            val = ("0", "1")[bool(val)] 
        else:
            val = state.dtype(val)
        resp = self._ask(state.cmd+" "+str(val))
        if self.debug:
            print("\tresp:"+resp)
        if resp[0] == '?':
            raise IOError("QC error: " + error_codes.get(int(resp[1]), resp))
        assert resp == 'ok'
        
    def read_pulse_state(self, chan, x):
        if chan in self.channel_names:
            chan = self.chan_name_dict[chan]
        assert 1 <= chan < self.num_channels
        
        if self.debug:
            print("read_pulse_state", chan, x)
        state = pulse_states_dict[x]
        resp = self._ask(":PULSE{}:{}?".format(chan, state.cmd))
        if resp[0] == '?':
            raise IOError("QC error: " + error_codes.get(resp[1], resp))        
        return state.dtype(resp)
    
    def write_pulse_state(self, chan, name, val):
        if self.debug:
            print("write_pulse_state", chan,  name, val)
        state = pulse_states_dict[name]
        
        if chan in self.channel_names:
            chan = self.chan_name_dict[chan]
        assert 1 <= chan < self.num_channels
        
        if state.dtype == bool:
            val = ("0", "1")[bool(val)] 
        else:
            val = state.dtype(val)
        resp = self._ask(":PULSE{}:{} {}".format(chan, state.cmd, str(val)))
        # b'ok\r\n'
        if self.debug:
            print("\tresp:"+resp)
        if resp[0] == '?':
            raise IOError("QC error: " + error_codes.get(int(resp[1]), resp))
        assert resp == 'ok'
    
    def get_pulse(self,chan=1):
        
        if chan in self.channel_names:
            chan = self.self.chan_name_dict[chan]
    
        if chan==0:
            return self.get_spulse()
        # assert 1 <= chan < self.num_channels
        
        self.chan[chan].state = bool(self._ask(":PULSE%i:STATE?" % chan))
        self.chan[chan].width = float(self._ask(":PULSE%i:WIDTH?" % chan))
        self.chan[chan].delay = float(self._ask(":PULSE%i:DELAY?" % chan))
        self.chan[chan].sync_to = self._ask(":PULSE%i:SYNC?" % chan)
        self.chan[chan].mux   = int(self._ask(":PULSE%i:MUX?" % chan))        
        self.chan[chan].polarity = self._ask(":PULSE%i:POLarity?" % chan)
        self.chan[chan].output_mode = self._ask(":PULSE%i:OUTP:MODE?" % chan)
        self.chan[chan].output_amplitude = float(self._ask(":PULSE%i:OUTP:AMPL?" % chan))
        self.chan[chan].mode = self._ask(":PULSE%i:CMODE?" % chan)
        self.chan[chan].burst_counter = int(self._ask(":PULSE%i:BCOunter?" % chan))
        self.chan[chan].duty_on_counter = int(self._ask(":PULSE%i:PCOunter?" % chan))
        self.chan[chan].duty_off_counter = int(self._ask(":PULSE%i:OCOunter?" % chan))
        self.chan[chan].wait_counter = int(self._ask(":PULSE%i:WCOunter?" % chan))
        self.chan[chan].trigger_source = self._ask(":PULSE%i:CTRIG?" % chan)
        self.chan[chan].gate_mode = self._ask(":PULSE%i:CGATe?" % chan)
        self.chan[chan].gate_active_logic = self._ask(":PULSE%i:CLOGic?" % chan)
    
        return self.chan[chan]
    
    def update_display(self):
        resp = self._ask(":DISP:UPDate?")
        assert resp == b"ok"

if __name__ == '__main__':
    from pprint import pprint

    #try:
    qc = QuantumComposer9530("COM11", debug=True)
    print(qc.get_idn())
    
    pprint( vars(qc) )
    for chan in qc.chan:
        pprint( vars(chan) )


    print("="*80)
    print(qc._ask(":SYSTem:STATe"))
    print("="*80)

    result = qc.read_spulse_state("state")
    print(result)    
    qc.write_spulse_state("state", False)
    result = qc.read_spulse_state("state")
    print("r1", result)    
    print("="*80)

    qc.update_display()
        

    #finally:
    #    qc.close()
    
    
"""
SCPI Command Tree:

    INSTrument          Subsystem
        CATalog             Query
        FULL                Query
        COMMands            Query
        NSELect             Integer
        SELect              List
        STATe               Boolean
    PULSe               SubSystem#
        STATe               Boolean
        WIDTh               Triple
        DELay               Triple
        IWIDth              Triple
        IDELay              Triple
        SYNC                List
        MUX                 Integer
        POLarity            List
        CMODe               List
        BCOunter            Double
        PCOunter            Double
        OCOunter            Double
        WCOunter            Double
        CGATe               List
        CLOGic              List
        CTRIg               List

        OUTPut              Subsystem
            MODe                List
            AMPLitude           Single

    SPULse              Subsystem
        STATe               Boolean
        PERiod              Triple
        MODe                List
        IRESet              Boolean
        BCOunter            Double
        PCOunter            Double
        OCOunter            Double
        CYCLe               Integer

        ICLock              Subsystem
            MODe                List
            RATe                Integer
            LEVel               Single
            OPTion              List
        OCLock              List

        GATe                Subsystem
            MODe                List
            LOGic               List
            EDGe                List
            LEVel               Single

        TRIGger             Subsystem
            MODe                List
            EDGe                List
            LEVel               Single

        COUNter             Subsystem
            STATe               Boolean
            CLear               List
            COUNt               List

    DISPlay             Subsystem
        MODe                Boolean
        BRIGhtness          Integer
        UPDate              Query

    SYSTem              Subsystem
        STATe               Query

        BEEPer              Subsystem
            STATe               Boolean
            VOLume              Integer

        COMMunicate         Subsystem

            SERial              Subsystem
                BAUD                List
                USB                 List
                ECHo                Boolean
        AUTorun             Boolean
        KLOCk               Boolean
        CAPS                Boolean
        VERSion             Query
        SERNumber           Query
        INFOrmation         Query
        NSID                Query
"""