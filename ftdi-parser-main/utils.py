import os

def get_data(filename, path = "data/"):
    # Matrix para armazenar os valores
    

    filename = path + filename
    # Leitura do arquivo de texto
    with open(filename, "r") as arquivo:
        title = arquivo.readline().strip().split(",")
        datas = [[] for i in range(len(title))]
        # Lê as linhas restantes do arquivo
        for linha in arquivo:
            # Divide a linha em comando e corrente
            linha = linha.strip().split(",")
            for i in range(len(datas)):
                datas[i].append(float(linha[i]))

    return datas

def Fourier(L, xs, fs, NUM_COEFFS = 80, num_fourier_points = None):
    # Compute A0
    a0  = 0
    for i in range(len(xs) - 1):
        # Trapezoidal integral
        f = (fs[i] + fs[i+1]) / 2
        dt = xs[i+1] - xs[i]
        a0 += f * dt
    a0 = a0 / L

    # Compute Coefficients
    aks, bks = [], []
    for k in range(1, NUM_COEFFS):
        ak = 0
        bk = 0
        dt = (xs[-1] - xs[0])/len(xs)
        for i in range(len(xs) - 1):
            f = (fs[i])
            # dt = xs[i+1] - xs[i]
            ak += (2/L) * f*np.cos((2*pi*k*xs[i])/L) * dt
            bk += (2/L) * f*np.sin((2*pi*k*xs[i])/L) * dt
        
        aks.append(ak)
        bks.append(bk)

    # Reconstruct the fourier function
    if num_fourier_points != None:
        xs = np.linspace(xs[0], xs[-1], num_fourier_points)

    fourier = []
    for i in range(len(xs)):
        f = a0
        x = xs[i]
        for k in range(len(aks)):
            f += aks[k]*np.cos((k+1)*2*pi*x/L) + bks[k]*np.sin((k+1)*2*pi*x/L)
        fourier.append(f)
    
    return fourier
