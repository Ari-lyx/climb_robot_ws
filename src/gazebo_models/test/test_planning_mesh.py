"""Planning mesh must remain closed and conservative after simplification."""
import math
import tempfile
import unittest
from collections import Counter
from pathlib import Path
from gazebo_models.world import planning_shell_mesh

class PlanningMeshTests(unittest.TestCase):
    def test_planning_shell_is_closed_and_conservative(self):
        with tempfile.TemporaryDirectory() as tmp:
            for half in (True,False):
                vertices,faces=planning_shell_mesh(Path(tmp)/'planning.stl',3.0,.06,half)
                self.assertLessEqual(len(faces),4608)
                edges=Counter()
                for ids in faces:
                    a,b,c=[vertices[i] for i in ids]
                    for i,j in zip(ids,ids[1:]+ids[:1]):edges[(i,j)]+=1
                    radii=[math.sqrt(sum(x*x for x in p)) for p in (a,b,c)]
                    if max(radii)-min(radii)>1e-8:continue # opening rim
                    u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
                    n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
                    distance=abs(sum(n[i]*a[i] for i in range(3)))/math.sqrt(sum(x*x for x in n))*3.0
                    if max(radii)<1.000001:
                        self.assertLessEqual(distance,3.0+1e-9)
                        self.assertGreater(distance,3.0-(.0145 if half else .0257))
                    else:self.assertGreaterEqual(distance,3.06-1e-9)
                for (a,b),count in edges.items():
                    self.assertEqual(count,1)
                    self.assertEqual(edges[(b,a)],1)


if __name__ == "__main__":
    unittest.main()
