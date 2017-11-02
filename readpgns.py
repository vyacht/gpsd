import csv

tr = 2

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


'''        <tr class="setting">
          <td>
            <p class="name"><strong>PGN</strong></p>
            <p class="description">Choose any random number.</p>
          </td>
          <td class="align-left">
              read
          </td>
          <td class="align-left">
              write
          </td>
        </tr>

        <p> / <span class="flag">&#x25CF;</span></p>
'''

if tr == 2:
    for row in pgns:
        if(str.isdigit(row['pgn'])):
            if((int(row['receive']) == 1) or (int(row['transmit']) == 1)):
                print('<tr class="setting">')
                print('    <td>')
                print('        <p class="name"><strong>' + row['pgn'] + '</strong></p>')
                print('        <p class="description">' + row['msg'] + '</p>')
                print('    </td>')
                print('    <td class="align-center">')
                print('    <p>')

                if(int(row['receive']) == 1):
                    print('    <span class="option">&#x25CF;</span>')
                else:
                    print('    -')
                print('/')
                if(int(row['transmit']) == 1):
                    print('    <span class="flag">&#x25CF;</span>')
                else:
                    print('    -')
                print('    </p>')
                print('    </td>')
                print('</tr>')
