import math
from typing import Tuple


class Ackermann:
    """
    Tools for calculations pertaining to ideal Ackermann steering geometry
    
    Parameters
    ----------
    w : float
        Width of wheelbase measured from centers of two wheels (m) 
    l : float
        Distance between front and rear axles (m)
    wheel_r : float
        Radius of the wheels (m)
    steering_ratio : float
        Ratio between input angle and ideal ackermann steer angle. NOT BEING USED ANYWHERE
    arm_wheel_angle : float
     	NOT BEING USED ANYWHERE
    """
    def __init__(self, w: float, l: float, wheel_r: float, steering_ratio: float):
        self.w, self.l = w, l
        self.w2 = 0.5*w
        self.wheel_r = wheel_r
        self.arm_wheel_angle = math.atan(self.w2*l)
        self.steering_ratio = steering_ratio

    def steer_to_wheel_turn(self, steer) -> Tuple[float, float]:
        """
        UNUSED FUNCTION
        """
        t1 = math.tan(steer/self.steering_ratio)
        return math.atan(self.l*t1/(self.l + 0.5*self.w*t1)), \
               math.atan(self.l*t1/(self.l - 0.5*self.w*t1))

    def lin_ang_to_steer_spin(self, lin: float, ang: float) -> Tuple[float, float, float]:
	"""
	Converts the linear velocity and angular velocity (steering) commands to a spin (angular velocity) value for the driving motor, and angles th1 and th2 as Ackermann steering angles
    
	Parameters
	----------
	lin : float
		linear velocity command
	ang : float
		angular velocity (steering) command
	
	Returns
	---------
	spin : float
		angular velocty calue for driving motor (not steering)
	th1 : float
		inner Achkermann steering angle
	th2 : float
		outer Achkermann steering angle
	
	"""
        spin = self.wheel_r*lin  # Driven wheel rotational speed. Uncertain about the calculation method
        R = lin/ang  # Turning radius
        th1, th2 = math.atan(self.l/(R-self.w2)), math.atan(self.l/(R+self.w2))  # Left and right wheel turning angles
        print('th12', th1, th2)
        return spin, th1, th2


class RackAndPinion:
    """
    Tools for calculations pertaining to a rack and pinion steering mechanism
    
    Parameters
    ----------
    w : float
    	Width of wheelbase measured from centers of two wheels (m)
    l : float
    	Distance between front and rear axles (m)
    wheel_r : float
    	Radius of the wheels (m)
    links : Tuple[float,float,float,float]
    	Lengths of each of the 4 links, where link 0 is the rack
    h : float
    	distance between the rack and the would-be front axle
    pinion_r : float
    	radius of pinion
    
    l12x : float 
    	the lengths of link 1 and link 2 in the direction parallel to the front and rear axles (m)
    th1 :  float
    	angle between link 1 and axle at rest
    th0 : float
    	angle between link 2 and axle at rest
    	
    
    """
    def __init__(self, w: float, l: float, wheel_r: float,
                 link_lengths: Tuple[float, float, float, float],
                 rack_axle_dist: float, pinion_r: float):
        """
        """
        # Specified quantities
        self.w, self.l = w, l
        self.wheel_r = wheel_r
        self.links = link_lengths  # Lengths of each of the 4 links, where link 0 is the rack
        self.h = rack_axle_dist  # distance between the rack and the would-be front axle
        self.pinion_r = pinion_r

        # Derived quantities
        self.l12x = (w - self.links[0])/2 - self.links[3]  # Width of linkage formed by links 1 and 2 ##not sure about this
        l12_2 = self.h**2 + self.l12x**2
        th12 = math.atan(self.h/self.l12x)
        th1 = th12 - math.acos((self.links[1]**2 + l12_2 - self.links[2]**2)/(2*self.links[1]*math.sqrt(l12_2)))
        # Angle between link 2 and axle at rest
        self.th0 = math.atan((self.h - self.links[1]*math.sin(th1))/(self.l12x - self.links[1]*math.cos(th1)))

    def disp_to_steer(self, rack_disp: float) -> float:
        """
        Calculate wheel steering angle (from +/-y axis) given rack displacement
        (in the +/-y direction)
        
        FUNCTION INCOMPLETE AND UNUSED
        
        """
        b = self.b0 - rack_disp
        d2 = self.h**2 + b**2
        th = math.atan(self.h/b) - math.acos((self.links[2]**2 + d2 - self.links[1]**2)/ \
                                             (2*self.links[2] * math.sqrt(d2)))
        return th - self.th0

    def steer_to_pinion_ang(self, steer: float) -> float:  # TODO: rename steer
        """
        Calculate rack displacement (from +/-y axis) given wheel steering angle
        (in the +/-y direction)
        WILL HAVE TO VERFITY THE AXIS
        
        Parameters
        ----------
        steer : float
        	the wheel steering angle
        
        Returns
        -------
        pinion_ang : float
        	amount of ratation required for pinion (in radains probably)
        """
        dth = self.th0 - steer
        print('th', dth, self.th0)
        l2x, l2y = self.links[2]*math.cos(dth), self.links[2]*math.sin(dth)
        print('l2', l2x, l2y)
        l1x = math.sqrt(self.links[1]**2 - (self.h - l2y)**2)
        rack_disp = self.l12x - l1x - l2x
        return rack_disp/self.pinion_r

    def wheel_axis_intercept(self, steer: Tuple[float, float]) -> Tuple[float, float]:
        """
        Calculate point at which the axis of rotation of both front wheels meet given 2 steering angles.
        Origin taken as center of rear axle
        
         FUNCTION UNUSED
        """
        if steer == (0, 0): return (math.inf, math.inf)  # No steering yields infinite points of intersection

        d1 = (self.w/2 - self.links[3])/math.sin(abs(steer[0]) - abs(steer[1]))
        n1 = math.sin(abs(steer[0]) + abs(steer[1]))
        n2, n3 = math.sin(abs(steer[0])), math.sin(abs(steer[1]))

        return n1*d1, 2*d1*n2*n3






        
        




