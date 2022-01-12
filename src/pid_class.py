
from control_msgs.msg import PidState


class pid_controller:
    def __init__(self):
        self.kp=0
        self.ki=0
        self.kd=0
        self.windup=0

        self.last_err=0
        self.err=0
        self.int_err=0

        self.output=0

        self.output_max=0
        self.output_min=0
        self.output_zero=0


    def update(self,err):
        self.err=err

        p_error=self.kp*self.err

        self.int_err=min(self.windup,max(self.int_err+self.err,self.windup*(-1.0)))
        i_error=self.ki*self.int_err

        d_error=self.kd*(self.err-self.last_err)

        self.output=p_error+i_error+d_error
        if abs(self.output)<self.output_zero:
            self.output=0
        else:
            self.output=min(self.output_max,max(self.output_min,self.output))

        pid_msg=PidState()

        pid_msg.error=self.err
        pid_msg.error_dot=(self.err-self.last_err)

        pid_msg.p_term=self.kp
        pid_msg.i_term=self.ki
        pid_msg.d_term=self.kd

        pid_msg.p_error=p_error
        pid_msg.i_error=i_error
        pid_msg.d_error=d_error

        pid_msg.i_max=self.windup
        pid_msg.i_min=self.windup*(-1.0)

        pid_msg.output=self.output

        self.last_err=self.err

        return pid_msg

    def setpid(self,kp,ki,kd):
        self.kp=kp
        self.ki=ki
        self.kd=kd