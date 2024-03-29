from scipy.optimize import fsolve
alpha_s = epsillon = 0.6
G_s = 900  # W/m^2
h_bar = 53.82  # W/m^2K
Tsi_celsius = -10  # Celsius
Tinf_celsius = 30  # Celsius
sigma = 5.67e-8  # W/m^2K^4
t1 = 5e-3  # m
t2 = 50e-3  # m
Ki = 0.026  # W/mK
Kp = 180  # W/mK
Tsi_kelvin = Tsi_celsius + 273.15
Tinf_kelvin = Tinf_celsius + 273.15
def equation(Tso):
    return alpha_s * G_s + h_bar * (Tso - Tinf_kelvin) - ((Tso - Tsi_kelvin) / (2 * t1 / Kp + t2 / Ki)) - epsillon * sigma * (Tso**4)
initial_guess = 300  # Kelvin
Tso_solution = fsolve(equation, initial_guess)
q_ref = (Tso_solution - Tsi_kelvin) / (2 * t1 / Kp + t2 / Ki)
print(q_ref)
