# This code provides a BRP algorithm for bottom-up side
import numpy as np
import time
import matplotlib.pyplot as plt
from PIL import Image
import numpy.ma as ma


def search_path_by_points(route, pos, npath=list(), ch_dots=list(), ind=1):
    for k in range(len(route), ind, -1):
        path = bresenhamBRP(route[ind - 1], route[k-1])
        brm = pos[path[:, 0], path[:, 1]]
        mnn = ma.is_masked(brm)

        if not mnn:
            break

        if k - 1  == ind:
            break

    ind = k
    for elem in path[:-1]:
        npath.append(elem)

    ch_dots.append(path[-1])
    if ind == len(route):
        return npath, ch_dots
    else:
        return search_path_by_points(route, pos, npath, ch_dots, ind)


def get_closest_yBRP(p, we):
    # Search closest point on y-axis
    dist = np.linalg.norm([we[:,1]] - p[1], axis=0)
    mind = np.min(dist)
    elms = we[dist == mind]
    dist = np.linalg.norm(elms - p, axis=1)
    dist = ma.masked_array(dist, mask=ma.getmask(elms)[:,0])
    ind_min = dist.argmin(axis=0, fill_value = 100)
    npp = (elms[ind_min]).astype(int)

    return npp


def get_closest_yspBRP(p, sp, we):
    # Search closest point on y-axis (additional)
    dist = np.linalg.norm([we[:,1]] - p[1], axis=0)
    mind = np.min(dist)
    elms = we[dist == mind]
    dist = np.linalg.norm(elms - sp, axis=1)
    dist = ma.masked_array(dist, mask=ma.getmask(elms)[:,0])
    ind_min = dist.argmin(axis=0, fill_value = 100)
    npp = (elms[ind_min]).astype(int)

    return npp


def get_closest_xBRP(p, we):
    # Search closest point on y-axis
    dist = np.linalg.norm([we[:,0]] - p[0], axis=0)
    mind = np.min(dist)
    elms = we[dist == mind]
    dist = np.linalg.norm(elms - p, axis=1)
    dist = ma.masked_array(dist, mask=ma.getmask(elms)[:,0])
    ind_min = dist.argmin(axis=0, fill_value = 100)
    npp = (elms[ind_min]).astype(int)

    return npp


def line_check(mp,sp,imn):
    brl = bresenhamBRP(sp, mp)
    brm = imn[brl[:, 0], brl[:, 1]]

    return ma.is_masked(brm)


def bresenhamBRP(s, g):
    x0, y0 = s[0], s[1]
    x1, y1 = g[0], g[1]

    dx = x1 - x0
    dy = y1 - y0

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, ysign, xsign, 0

    D = 2*dy - dx
    y = 0

    way = []
    for x in range(dx + 1):
        way.append([x0 + x*xx + y*yx, y0 + x*xy + y*yy])
        if D >= 0:
            y += 1
            D -= 2*dx
        D += 2*dy

    return np.array(way)


def line_search(koef, ire_dots, sp, mp, fp, dots, imn, way, mp_prev, count=0, rev_flag=0, flag=0):
    '''
    Main BRP cycle

    sp   - start point
    mp   - middle point
    fp   - goal point
    dots - filtered net weights
    imn  - NaNs masked image
    '''
    count += 1

    if len(way) > 0 and ((mp == mp_prev).all() or flag ):
            count_agent = 0
            flag = 0
            k = 0
            r = 0

            mask_copy = ma.getmask(imn).copy()
            nmp = sp.copy()
            nmp[1] += koef
            nmp  = get_closest_yspBRP(nmp, sp, dots)
            mnn = line_check(nmp,sp,imn)

            if mnn:
                r = 1 if len(way) > 2 else 0
                mask = mask_copy.copy()
                mask[tuple(way[-1])] = True

                if r:
                    mask[tuple(way[-2])] = False

                imn_new = ma.masked_array(ire_dots,mask=mask)
                re_dots = imn_new.reshape(-1,2)
                nmp = get_closest_xBRP(sp,re_dots)
                mnn = line_check(nmp,sp,imn)

                if mnn:
                    mask = mask_copy.copy()
                    mask[tuple(way[-1])] = True
                    if r:
                        mask[tuple(way[-2])] = True

                    imn_new = ma.masked_array(ire_dots,mask=mask)
                    re_dots = imn_new.reshape(-1,2)

                    nmp  = get_closest_yBRP(sp, re_dots)
                    mnn  = line_check(nmp,sp,imn)

                    if mnn:
                        k = 1
                        mask = mask_copy.copy()
                        mask[tuple(way[-1])] = True

                        if r:
                           mask[tuple(way[-2])] = False
                           way = way[:-1]

                        imn = ma.masked_array(ire_dots,mask=mask)
                        dots = imn.reshape(-1,2)

                        sp = way[-1]
                        mp = fp

            if not k:
                mask = mask_copy.copy()
                mask[tuple(way[-1])] = True

                if r:
                    mask[tuple(way[-2])] = True

                imn = ma.masked_array(ire_dots,mask=mask)
                dots = imn.reshape(-1,2)
                way.append(nmp)
                sp = nmp
                mp = fp

            if count > 500 and rev_flag == 0:
                rev_flag = 1
                sp = way[0]

    brl = bresenhamBRP(sp, mp)
    brm = imn[brl[:, 0], brl[:, 1]]
    mnn = ma.is_masked(brm)

    if mnn == True:
        mp_prev = mp
        mp = (sp + mp) * 0.5

        mp = get_closest_yBRP(mp, dots)
        mp = mp.astype(int)


        return line_search(koef, ire_dots, sp, mp, fp, dots, imn, way, mp_prev, count, rev_flag, flag)

    else:
        mp = np.array(mp)
        mp_prev = mp

        _, qu = np.unique(way, return_counts=True)

        if qu[qu > 50].any():
            flag = 1

        if (mp == way[-1]).all():
            flag = 1
        else:
            way.append(mp)

        if (mp == fp).all():
            return way, imn, count

        sp = mp
        return line_search(koef,ire_dots, sp, fp, fp, dots, imn, way, mp_prev, count, rev_flag, flag)


def conv_image_rev(img, t):
    li = []
    for i in range(img.shape[0]):
        for k in range(img.shape[1]):
            if img[i, k] == t :
                li.append([k,i])
    return np.array(li)


def create_mask(ni, inv):
    Ni, Nk = ni.shape[0], ni.shape[1]
    tli = []

    if inv == True:
        param = 0
    else:
        param = 1

    for i in range(Ni):
        li = []
        for k in range(Nk):

            if ni[i,k] == 0:
                ni[i,k] = param
            else:
                ni[i,k] = int(not param)

            li.append([ni[i,k],ni[i,k]])
        tli.append(li)

    return np.array(tli)


def main():
    im_name = 'MAP3.png'
    invert = 1
    n = 50

    stp = np.array([49, 5])
    fip = np.array([3, 41])

    fni = np.array(Image.open(im_name))

    fni = np.swapaxes(fni,0,1)
    imi = conv_image_rev(fni.T,invert)
    ni  = create_mask(fni, 1)

    x, y = np.meshgrid(np.linspace(0,49, n), np.linspace(0,49, n))
    pos = np.stack([x, y], axis=2)

    pos = ma.masked_array(pos, mask=ni)

    area = pos.reshape(-1,2)

    pos[:,:,[0,1]] = pos[:,:,[1,0]]
    dots_copy = pos.data.copy()

    koef = 1 if stp[1] <= fip[1] else -1
    start_time = time.perf_counter_ns()
    route_51,imn,count = line_search(koef, dots_copy, stp, fip, fip, area, pos, list([stp]), np.zeros(2))

    route_51.insert(0, stp)
    route_51 = np.array(route_51)

    nw_path, ch_dots = search_path_by_points(route_51, pos)
    nw_path.insert(-1,fip)

    print("--- %s seconds ---" % ((time.perf_counter_ns() - start_time) / 1e9))

    imn_comp = (ma.getmask(pos) == ma.getmask(imn)).all(-1)
    row, cols = np.where(imn_comp == False)
    nw_path = np.array(nw_path)
    common_elements = np.array(ch_dots)

    fig = plt.figure()

    ax = fig.add_subplot(121)
    plt.gca().set_aspect('equal')

    plt.title("(a)")
    ax.set_facecolor('whitesmoke')

    ax.scatter(stp[0], stp[1], color='yellow', s=40, edgecolors='black', zorder=70)
    ax.scatter(fip[0], fip[1], color='purple',  s=40, edgecolors='black', zorder=70)

    ax.scatter(route_51[:,0],route_51[:,1], s=25, color='red', zorder=60, edgecolors='black')

    ax.scatter(imi[:,0],imi[:,1], s=70, marker='s', color='black', zorder=1)
    ax.scatter(row, cols, s=70, marker='s', color='mediumaquamarine', zorder=59,alpha=0.7)
    ax.legend(['Start', 'Goal', 'Waypoint', 'Obstacle', 'Blocked Point'], loc='upper right',  framealpha=0.95, labelcolor='black').set_zorder(102)

    ax.set_xlim(0, 49)
    ax.set_ylim(0, 49)
    ax.set_xlabel('$X$')
    ax.set_ylabel('$Y$')

    major_ticks = np.arange(0, 49, 10)
    minor_ticks = np.arange(0, 49, 1)

    ax.set_xticks(major_ticks)
    ax.set_xticks(minor_ticks, minor=True)
    ax.set_yticks(major_ticks)
    ax.set_yticks(minor_ticks, minor=True)

    ax.grid(which='both')
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)

    ax = fig.add_subplot(122)
    plt.gca().set_aspect('equal')
    plt.title("(b)")

    ax.scatter(stp[0], stp[1], color='yellow', s=40, edgecolors='black', zorder=70)
    ax.scatter(fip[0], fip[1], color='purple',  s=40, edgecolors='black', zorder=70)
    ax.scatter(common_elements[:,0],common_elements[:,1], s=30, color='red', zorder=60, edgecolors='black')
    ax.scatter(imi[:,0],imi[:,1], s=70, marker='s', color='black', zorder=1)
    ax.scatter(nw_path[:,0],nw_path[:,1], marker='s', s=60, color='cornflowerblue', zorder=50, edgecolors='black')
    ax.legend(['Start', 'Goal', 'Waypoint', 'Obstacle', 'Path'], loc='upper right', framealpha=0.95, labelcolor='black').set_zorder(102)

    ax.set_xlim(0, 49)
    ax.set_ylim(0, 49)
    ax.set_xlabel('$X$')
    ax.set_ylabel('$Y$')

    major_ticks = np.arange(0, 49, 10)
    minor_ticks = np.arange(0, 49, 1)

    ax.set_xticks(major_ticks)
    ax.set_xticks(minor_ticks, minor=True)
    ax.set_yticks(major_ticks)
    ax.set_yticks(minor_ticks, minor=True)

    ax.grid(which='both')
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)

    plt.show()

if __name__ == '__main__':
    main()
