import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt


def main():
    xmis = np.array([0.0, 0.0, 10000.0])
    xdotmis = np.array([0.0, 1.0, 0.0])

    xtgt = np.array([0.0, 10000.0, 5000.0])
    xdottgt = np.array([300.0, 0.0, 0.0])

    # constant g target maneuver
    xddottgt = np.array([0.0, 0.0, 30.0]) # 0g

    xdotmis = 4.0 * norm(xdottgt) * xdotmis / norm(xdotmis) # normalize missile velocity and then multiply by some relative missile speed to target.
    # print(xdotmis)

    xmis_xhist = []
    xmis_yhist = []
    xmis_zhist = []

    xtgt_xhist = []
    xtgt_yhist = []
    xtgt_zhist = []

    t_hist = []
    range_hist = []
    accel_hist = []

    dt = 0.001
    t = 0
    tgo = 0
    rdot = -1 # some initial negative number
    range_current = norm(xtgt - xmis)
    range_prev = norm(xtgt - xmis) + 1

    # simulation
    while range_current - range_prev < 0:

        range_prev = range_current
        xmis_xhist.append(xmis[0])
        xmis_yhist.append(xmis[1])
        xmis_zhist.append(xmis[2])

        xtgt_xhist.append(xtgt[0])
        xtgt_yhist.append(xtgt[1])
        xtgt_zhist.append(xtgt[2])
        t_hist.append(t)

        xddotmis = pronav(xmis, xdotmis, xtgt, xdottgt)
        accel_hist.append(xddotmis[0])

        xdotmis = xdotmis + xddotmis * dt

        xdottgt = xdottgt + xddottgt * dt

        xmis = xmis + xdotmis * dt
        xtgt = xtgt + xdottgt * dt

        # print(((xdottgt[0]-xdotmis[0]) + (xdottgt[1]-xdotmis[1])))
        range = norm(xtgt-xmis)
        # print(range)
        range_current = range

        range_hist.append(range)



        t += dt


    print("Final Miss Distance: ", range_current, range_prev)
    print("Final Time of Flight: ", t)

    # plot 3d
    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(xmis_xhist, xmis_yhist, xmis_zhist, 'b-')
    ax.plot(xtgt_xhist, xtgt_yhist, xtgt_zhist, 'r--')

    plt.show()

    input("Press Enter to Continue...")

    # plt.subplot(2,2,2)
    # plt.plot(xmis_xhist, xmis_yhist, 'b-')
    # plt.plot(xtgt_xhist, xtgt_yhist, 'r--')
    # plt.subplot(2,2,4)
    # plt.plot(xmis_yhist, xmis_zhist, 'b-')
    # plt.plot(xtgt_yhist, xtgt_zhist, 'r--')


# calculate guidance commands
def pronav(xmis, xdotmis, xtgt, xdottgt):

  N = 3

  Rtm = xtgt - xmis
  Rtm_hor = Rtm[0:2]
  Rtm1 = Rtm[0]
  Rtm2 = Rtm[1]
  Rtm3 = Rtm[2]

  Vtm = xdottgt - xdotmis
  Vtm_hor = Vtm[0:2]
  Vtm1 = Vtm[0]
  Vtm2 = Vtm[1]
  Vtm3 = Vtm[2]

  uvRtm = Rtm / norm(Rtm)
  uvVm = xdotmis / norm(xdotmis)
  uvVtm = Vtm / norm(Vtm)

  lam = np.arccos( np.dot(uvRtm, uvVm) )
  lambdadot = - 1 / np.sin(lam) * np.dot(uvVtm, uvVm)

  Vc = - ( Rtm1 * Vtm1 + Rtm2*Vtm2 + Rtm3*Vtm3 ) / ( norm(Rtm) )

  nc = N * Vc * lambdadot

  turn_vector = np.cross(uvVm, uvRtm)
  steering_vector = np.cross(turn_vector, uvVm)


  return nc * steering_vector

def aug_pronav(xmis, xdotmis, xtgt, xdottgt):
  return



if __name__ == '__main__':
  main()


