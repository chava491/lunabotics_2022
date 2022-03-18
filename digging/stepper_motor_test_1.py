import subprocess
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper
 
def ticcmd(*args):
  return subprocess.check_output(['ticcmd'] + list(args))
 
status = load(ticcmd('-s', '--full'), Loader=Loader)
 
position = status['Current position']
print("Current position is {}.".format(position))
 
new_target = -200 if position > 0 else 200
print("Setting target position to {}.".format(new_target))
ticcmd('--exit-safe-start', '-p', str(new_target))

while position != new_target:
  try:
    ticcmd('--resume')
    status = load(ticcmd('-s', '--full'), Loader=Loader)
    position = status['Current position']
  except:
    print("Unable to resume") 

ticcmd('--deenergize')

print("Current position is {}.".format(position))