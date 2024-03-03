__PACKAGES__= [
  'base',
  'col_mi',
  'const',
  'cv',
  'kdl_kin',
  'pointcloud',
  'robot',
  'rs',
  'viz',
  ]

from base         import *
try:  from col_mi       import *
except ImportError as e:  print str(e)
from const        import *
from cv           import *
try:  from kdl_kin      import *
except ImportError as e:  print str(e)
from pointcloud   import *
from robot        import *
from rs           import *
from viz          import *
