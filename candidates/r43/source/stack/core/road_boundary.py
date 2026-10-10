"""Mapped road-body diagnostics; never infer asphalt outside supplied polygons."""
import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.ops import unary_union
from .shared_border import _shared_border_vertices


class RoadBoundary:
    def __init__(self, net):
        polygons=[]
        self.invalid=[]
        self.seams=[]
        lanes={str(l.lane_id):l for l in net.discretelanes}
        replacement,self.shared_borders=_shared_border_vertices(lanes)
        for key,lane in lanes.items():
            left=np.asarray(replacement.get(key,lane.left_vertices),float)
            right=np.asarray(lane.right_vertices,float)
            if len(left)<2 or len(right)<2:
                self.invalid.append(key)
                continue
            polygon=Polygon(np.vstack((left,right[::-1])))
            if not polygon.is_valid or polygon.area<=0.:
                self.invalid.append(key)
                continue
            polygons.append(polygon)
            for successor in lane.successor or ():
                other=lanes.get(str(successor))
                if other is None or key not in [str(v) for v in (other.predecessor or ())]:
                    continue
                first=np.asarray([left[-1],right[-1]])
                last=np.asarray([other.left_vertices[0],other.right_vertices[0]])
                gap=float(np.linalg.norm(first-last,axis=1).max())
                if 0.<gap<=.001:
                    patch=Polygon([first[0],first[1],last[1],last[0]])
                    if patch.is_valid and patch.area>0.:
                        polygons.append(patch)
                        self.seams.append(dict(source=key,target=str(successor),gap=gap))
        self.area=unary_union(polygons) if polygons else None
        if self.area is not None: shapely.prepare(self.area)

    def inspect(self,states,length,width,terminal=None):
        shape=states.shape[:-1]
        if self.area is None:
            return {'status':'unknown_missing_map','checked':False,'first_outside_step':None}
        flat=states.reshape(-1,states.shape[-1])
        h=flat[:,2]
        x,y=flat[:,0],flat[:,1]
        corners=[]
        for a,b in ((1,1),(1,-1),(-1,-1),(-1,1)):
            corners.append(np.column_stack((x+a*.5*length*np.cos(h)-b*.5*width*np.sin(h),
                                            y+a*.5*length*np.sin(h)+b*.5*width*np.cos(h))))
        bodies=shapely.polygons(np.stack(corners,axis=1))
        covered=np.asarray(shapely.covers(self.area,bodies)).reshape(shape)
        valid=np.ones(shape,dtype=bool)
        if terminal is not None:
            valid=np.arange(shape[-1])[None,:]<=terminal[:,None]
        outside=(~covered)&valid
        first=np.where(outside.any(-1),outside.argmax(-1),-1)
        return dict(status='mapped_body_on_grid',checked=True,first_outside_step=first,
                    outside_count=outside.sum(-1),covered=covered,
                    map_invalid_count=len(self.invalid),seam_count=len(self.seams),
                    reconciled_shared_borders=len(self.shared_borders))

    def outside_area(self,states,length,width):
        if self.area is None: return np.full(states.shape[:-1],np.inf)
        x,y,h=states[...,0],states[...,1],states[...,2]
        corners=[]
        for a,b in ((1,1),(1,-1),(-1,-1),(-1,1)):
            corners.append(np.stack((x+a*.5*length*np.cos(h)-b*.5*width*np.sin(h),
                                     y+a*.5*length*np.sin(h)+b*.5*width*np.cos(h)),axis=-1))
        bodies=shapely.polygons(np.stack(corners,axis=-2))
        flat=bodies.reshape(-1)
        outside=~np.asarray(shapely.covers(self.area,flat))
        areas=np.zeros(len(flat))
        if np.any(outside):
            areas[outside]=shapely.area(shapely.difference(flat[outside],self.area))
        return areas.reshape(bodies.shape)

    def recovers_initial_exterior(self,states,length,width,initial_area,terminal=None):
        if self.area is None or initial_area <= 0.:
            return False
        areas=self.outside_area(states,length,width)
        end=len(areas)-1 if terminal is None else min(int(terminal),len(areas)-1)
        # A supplied initial overhang is recoverable only without growing it
        # and with the body fully back on the map within the checked prefix.
        return bool(np.max(areas[:end+1]) <= initial_area+1e-7 and areas[end] <= 1e-7)
