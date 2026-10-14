"""Régulateur PID commun : intégrale bornée et dérivée filtrée."""



class PIDController:
    """
    PID simple avec :
      - anti-windup par clamping de l'intégrale
      - filtre dérivé (filtre passe-bas 1er ordre, coeff alpha)
      - reset() pour réinitialiser à chaque changement d'état FSM
    """
    def __init__(self, kp, ki, kd,
                 out_min=-1.0, out_max=1.0,
                 integrator_max=0.5,
                 derivative_alpha=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integrator_max = integrator_max
        self.alpha = derivative_alpha  # filtre dérivé : plus petit = plus lissé

        self._integral = 0.0
        self._prev_error = 0.0
        self._deriv_filtered = 0.0
        
        

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._deriv_filtered = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0

        # Terme P
        p = self.kp * error

        # Terme I avec anti-windup (clamping)
        self._integral += error * dt
        self._integral = max(-self.integrator_max,
                             min(self.integrator_max, self._integral))
        i = self.ki * self._integral

        # Terme D filtré (passe-bas pour limiter amplification du bruit)
        raw_deriv = (error - self._prev_error) / dt
        self._deriv_filtered = (self.alpha * raw_deriv
                                + (1.0 - self.alpha) * self._deriv_filtered)
        d = self.kd * self._deriv_filtered
        self._prev_error = error

        # Sortie bornée
        out = p + i + d
        return max(self.out_min, min(self.out_max, out))
