__PACKAGES__= [
  'base',
  'col_mi',
  'const',
  'kdl_kin',
  'robot',
  'viz',
  ]
__PACKAGES_ROBOT__= [
  'rbt_bxtr',
  'rbt_bxtrN',
  'rbt_dxlg',
  'rbt_pr2',
  'rbt_rqnb',
  ]

from base         import *
from col_mi       import *
from const        import *
from kdl_kin      import *
from robot        import *
from viz          import *

try:  from rbt_bxtr import *
except ImportError as e:  print str(e)
except Exception as e:  print 'Could not import rbt_bxtr as:',type(e),str(e)
try:  from rbt_bxtrN import *
except ImportError as e:  print str(e)
except Exception as e:  print 'Could not import rbt_bxtrN as:',type(e),str(e)
try:  from rbt_dxlg import *
except ImportError as e:  print str(e)
except Exception as e:  print 'Could not import rbt_dxlg as:',type(e),str(e)
try:  from rbt_pr2 import *
except ImportError as e:  print str(e)
except Exception as e:  print 'Could not import rbt_pr2 as:',type(e),str(e)
try:  from rbt_rqnb import *
except ImportError as e:  print str(e)
except Exception as e:  print 'Could not import rbt_rqnb as:',type(e),str(e)
