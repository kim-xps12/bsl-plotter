def write_a(t, t_start, pos):
    margin_x = 2
    tr = t - t_start
    start_x, start_y = pos
    x = start_x + margin_x
    y = start_y
    
    if tr >= 0:
        if t <= 2:
            x = 10 * tr + margin_x + start_x
            y = 20 * tr + start_y
        elif tr <= 4:
            x = 10 * tr + margin_x + start_x
            y = -20 * tr + start_y + 80
        elif tr <= 4.5:
            x = -10 * tr + margin_x + start_x + 80
            y = 20 * tr + start_y - 80
        elif tr <= 6.53:
            x = -15 * tr + margin_x + start_x + 103
            y = 10 + start_y
    return x, y

