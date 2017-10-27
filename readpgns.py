import csv

tr = 1

f = open('www/pgns.csv', 'r')
pgns = csv.DictReader(f, delimiter=',')

if tr == 0:
    for row in pgns:
        if(str.isdigit(row['pgn'])):
            if(int(row['pgn']) > 0):
                pgns = row['pgn']
                if(len(pgns) == 5):
                    pgns = '0' + pgns
                    print('static gps_mask_t hnd_' \
                          + pgns \
                          + '(unsigned char *bu, int len, struct PGN *pgn, struct gps_device_t *session);')

if tr == 1:
    for row in pgns:
        if(str.isdigit(row['pgn'])):
            pgns = row['pgn']
            pgn0 = row['pgn']
            if(len(pgns) == 5):
                pgns = ' ' + pgns
            if(len(pgn0) == 5):
                pgn0 = '0' + pgn0
            if(len(pgns) == 1):
                pgns = '     ' + pgns
            print('    {' + pgns + \
                  ', ' + row['fast'] +  \
                  ', ' + row['type'] + \
                  ', ' + row['receive'] + \
                  ', ' + row['transmit'] + \
                  ', hnd_' + pgn0 + \
                  ', "' + row['msg'] + '"},')
