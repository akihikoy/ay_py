__PACKAGES__= [
  'base',
  'col_mi',
  'const',
  'kdl_kin',
  'pointcloud',
  'robot',
  'viz',
  ]

from base         import *
try:  from col_mi       import *
except ImportError as e:  print str(e)
from const        import *
try:  from kdl_kin      import *
except ImportError as e:  print str(e)
from pointcloud   import *
from robot        import *
from viz          import *
