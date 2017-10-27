import re

#with open('driver_vyspi.c', 'r') as myfile:
with open('pseudon2k.c', 'r') as myfile:
    data=myfile.read()

# line = "/** \\brief this is a test \r\n * test more line \r\n */"

#matchObj = re.match( r'/\*\*(?:.|[\r\n])*?\*/', data, re.M|re.I)
matchObj = re.findall( r'/\*\*(?:.|[\r\n])*?\*/', data, 0)

if matchObj:
   for o in matchObj:
      s = o.splitlines()
      txt = ""
      for ss in s:
         su = re.sub( r'^\s*/\*\*\s*|^\s*\*/|^\s*\*\s*', '', ss, 0)
         if len(txt) > 0:
             txt = txt + "\n"
         txt = txt + su

      m = re.match( r'\\PGN (.+)', txt, re.DOTALL)
      if m:
          print "PGN " + m.group(1)

      m = re.match( r'\\TOPGN (.+)', txt, re.DOTALL)
      if m:
          print "PGN " + m.group(1)
      
