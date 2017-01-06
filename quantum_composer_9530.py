import serial

class QCChannel(object):
    def __init__(self,cnum, name):
        self.cnum = cnum
        self.name = name

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
        if self.debug: print "cmd: ", repr(cmd)
        self.ser.write(cmd + "\r\n")
        
        resp = self.ser.readline()
        if self.debug: print "resp:", repr(resp)
        return resp.strip()

    def get_idn(self):
        resp = self._ask("*IDN?")
        
        "QC,9534,02311,1.2.2-1.2.0"
        
        resp = resp.split(',')
        assert resp[0] == 'QC'
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
        self.spulse.gate_level = self._ask(":SPULSE:GATE:LEVel?")

        self.spulse.trigger_mode = self._ask(":SPULSE:TRIG:MODE?")
        self.spulse.trigger_edge = self._ask(":SPULSE:TRIG:EDGE?")
        self.spulse.trigger_level = self._ask(":SPULSE:TRIG:LEVel?")
        
        return self.spulse
        
    
    def get_pulse(self,chan=1):
    
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
        assert resp == "ok"

if __name__ == '__main__':
    from pprint import pprint

    #try:
    qc = QuantumComposer9530("COM1", debug=True)
    print qc.get_idn()

    qc.update_display()
        
    pprint( vars(qc) )
    for chan in qc.chan:
        pprint( vars(chan) )

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