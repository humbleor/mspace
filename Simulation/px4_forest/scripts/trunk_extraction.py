"""Normal-aware tree reconstruction from accumulated PCD observations."""
import numpy as np
from scipy.optimize import least_squares, linear_sum_assignment
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

DEFAULTS = dict(voxel=.06, min_height=1.5, min_points=45, min_radius=.055,
                max_radius=.55, max_fit_error=.05, max_colliders=500,
                normal_neighbors=20, normal_curvature=.09, normal_z=.65,
                normal_agreement=.65, outlier_std=2.5, slice_height=.5,
                cluster_tolerance=.22, slice_min_points=12, min_angular_bins=4,
                ransac_iterations=80, max_tilt_deg=35., max_slice_gap=1.,
                radius_change=.08, segment_length=1.5, bend_threshold=.045,
                ground_search=1.8, ground_max_extension=1.5,
                ground_max_error=.10, branch_cell=1., min_branch_length=1.2,
                min_branch_radius=.06, max_branch_radius=.22,
                branch_join_gap=.45, attachment_gap=.12,
                sparse_slice_height=.9, sparse_slice_min_points=8,
                sparse_min_height=2., sparse_angular_bins=3,
                min_track_occupancy=.5, sparse_max_slice_gap=1.5,
                sparse_normal_curvature=.18, sparse_spacing_factor=2.)


def surface_features(points, cfg, return_spacing=False):
    tree=cKDTree(points); normals=np.zeros_like(points)
    curvature=np.ones(len(points)); density=np.zeros(len(points))
    for start in range(0,len(points),12000):
        q=points[start:start+12000]
        d,ids=tree.query(q,k=min(int(cfg['normal_neighbors']),len(points)),workers=4)
        v=points[ids]; v=v-v.mean(axis=1,keepdims=True)
        vals,vec=np.linalg.eigh(np.einsum('nki,nkj->nij',v,v))
        normals[start:start+len(q)]=vec[:,:,0]
        curvature[start:start+len(q)]=vals[:,0]/np.maximum(vals.sum(axis=1),1e-12)
        density[start:start+len(q)]=d[:,1:].mean(axis=1)
    keep=density<=density.mean()+cfg['outlier_std']*density.std()+1e-9
    return (normals,curvature,keep,density) if return_spacing else (normals,curvature,keep)


def groups(points, tolerance):
    if not len(points):return []
    pairs=cKDTree(points).query_pairs(tolerance,output_type='ndarray')
    if len(pairs):
        graph=coo_matrix((np.ones(len(pairs)),(pairs[:,0],pairs[:,1])),shape=(len(points),len(points)))
        _,labels=connected_components(graph,directed=False)
    else:labels=np.arange(len(points))
    order=np.argsort(labels); _,counts=np.unique(labels,return_counts=True)
    return np.split(order,np.cumsum(counts)[:-1])


def angular_bins(radial, axis):
    u=np.cross(axis,[1.,0,0] if abs(axis[0])<.8 else [0.,1,0]);u/=np.linalg.norm(u)
    v=np.cross(axis,u)
    theta=np.arctan2(radial@v,radial@u)
    return len(np.unique(np.floor((theta+np.pi)*12/(2*np.pi)).astype(int)%12))


def slice_circle(p,n,cfg,rng):
    xy=p[:,:2]; best=None; best_count=0
    for _ in range(int(cfg['ransac_iterations'])):
        if rng.random()<.5:
            ia,ib=rng.choice(len(p),2,replace=False);a,b=xy[[ia,ib]]
            na,nb=n[[ia,ib],:2];mat=np.column_stack([na,-nb])
            if abs(np.linalg.det(mat))<.05:continue
            center=a+na*np.linalg.solve(mat,b-a)[0]
            radius=(np.linalg.norm(a-center)+np.linalg.norm(b-center))/2
        else:
            a,b,c=xy[rng.choice(len(p),3,replace=False)]
            mat=2*np.array([b-a,c-a])
            if abs(np.linalg.det(mat))<1e-6:continue
            center=np.linalg.solve(mat,np.array([b@b-a@a,c@c-a@a]))
            radius=np.linalg.norm(a-center)
        if not cfg['min_radius']<=radius<=cfg['max_radius']:continue
        radial=xy-center;dist=np.linalg.norm(radial,axis=1)
        align=np.abs(np.sum(radial*n[:,:2],axis=1))/np.maximum(dist,1e-9)
        good=(abs(dist-radius)<=cfg['max_fit_error'])&(align>=cfg['normal_agreement'])
        if good.sum()>best_count:best_count=int(good.sum());best=(center,radius,good)
    if best is None or best_count<cfg['slice_min_points']:return None
    center,radius,good=best
    local=np.linalg.norm(xy-center,axis=1)<=radius+cfg['max_fit_error']*2
    if best_count<.5*local.sum():return None
    center,radius,good=best
    fit=least_squares(lambda x:np.linalg.norm(xy[good]-x[:2],axis=1)-x[2],
                      [*center,radius],loss='soft_l1',f_scale=.02,max_nfev=40)
    center,radius=fit.x[:2],float(fit.x[2])
    radial=np.column_stack([xy-center,np.zeros(len(p))]);dist=np.linalg.norm(radial,axis=1)
    good=(abs(dist-radius)<=cfg['max_fit_error'])&(abs(np.sum(radial*n,axis=1))/np.maximum(dist,1e-9)>=cfg['normal_agreement'])
    if not cfg['min_radius']<=radius<=cfg['max_radius'] or good.sum()<cfg['slice_min_points']:return None
    if angular_bins(radial[good],[0.,0,1])<cfg['min_angular_bins']:return None
    return dict(center=np.r_[center,np.median(p[good,2])],radius=radius,points=p[good],normals=n[good])


def associate_slices(slices,cfg):
    tracks=[]; slope=np.tan(np.deg2rad(cfg['max_tilt_deg']))
    for candidates in slices:
        if not candidates:continue
        z=np.median([c['center'][2] for c in candidates])
        active=[t for t in tracks if 0<z-t[-1]['center'][2]<=cfg['max_slice_gap']+.05]
        if active:
            cost=np.full((len(active),len(candidates)),1e6)
            for i,t in enumerate(active):
                last=t[-1];dz=z-last['center'][2];velocity=np.zeros(2)
                if len(t)>1:velocity=(last['center'][:2]-t[-2]['center'][:2])/(last['center'][2]-t[-2]['center'][2])
                predicted=last['center'][:2]+velocity*dz
                for j,c in enumerate(candidates):
                    step=np.linalg.norm(c['center'][:2]-last['center'][:2])
                    delta=np.linalg.norm(c['center'][:2]-predicted)
                    if step<=slope*dz+.08 and abs(c['radius']-last['radius'])<=cfg['radius_change']:
                        cost[i,j]=delta+abs(c['radius']-last['radius'])
            ii,jj=linear_sum_assignment(cost);used=set()
            for i,j in zip(ii,jj):
                if cost[i,j]<1e6:active[i].append(candidates[j]);used.add(j)
            tracks.extend([[c] for j,c in enumerate(candidates) if j not in used])
        else:tracks.extend([[c] for c in candidates])
    return tracks


def cylinder_fit(p,n,center,slope,radius,cfg):
    zref=float(center[2]);max_slope=np.tan(np.deg2rad(cfg['max_tilt_deg']))
    def radial(x):
        axis=np.r_[x[2:4],1.];axis/=np.linalg.norm(axis)
        d=p-np.r_[x[:2],zref];r=d-(d@axis)[:,None]*axis
        return axis,r,np.linalg.norm(r,axis=1)
    def residual(x):
        axis,r,dist=radial(x)
        agreement=np.abs(np.sum(n*r,axis=1))/np.maximum(dist,1e-9)
        return np.r_[dist-x[4],.025*(1-agreement)]
    x=np.r_[center[:2],np.clip(slope,-max_slope+.001,max_slope-.001),radius]
    fit=least_squares(residual,x,bounds=([-np.inf,-np.inf,-max_slope,-max_slope,cfg['min_radius']],
                      [np.inf,np.inf,max_slope,max_slope,cfg['max_radius']]),loss='soft_l1',f_scale=.02,max_nfev=80)
    axis,r,dist=radial(fit.x)
    good=(abs(dist-fit.x[4])<=cfg['max_fit_error'])&(np.abs(np.sum(n*r,axis=1))/np.maximum(dist,1e-9)>=cfg['normal_agreement'])
    if good.sum()<cfg['min_points'] or good.mean()<.65 or axis[2]<np.cos(np.deg2rad(cfg['max_tilt_deg'])):return None
    if angular_bins(r[good],axis)<cfg['min_angular_bins']:return None
    return dict(point=np.r_[fit.x[:2],zref],axis=axis,radius=float(fit.x[4]),good=good,
                fit_median_error_m=float(np.median(abs(dist[good]-fit.x[4]))),
                normal_agreement=float(np.median(np.abs(np.sum(n[good]*r[good],axis=1))/dist[good])),
                support_points=int(good.sum()))


def axis_at(fit,z):
    return fit['point']+fit['axis']*((z-fit['point'][2])/fit['axis'][2])


def ground_plane(points,normals,curvature,base,radius,cfg,tree):
    ids=np.asarray(tree.query_ball_point(base[:2],cfg['ground_search']),dtype=int)
    if len(ids)<20:return None
    q=points[ids];radial=np.linalg.norm(q[:,:2]-base[:2],axis=1)
    q=q[(radial>radius+.15)&(abs(normals[ids,2])>.85)&(curvature[ids]<cfg['normal_curvature'])]
    if len(q)<20:return None
    keys=np.floor(q[:,:2]/.35).astype(int);_,inverse=np.unique(keys,axis=0,return_inverse=True)
    lows=np.full(inverse.max()+1,np.inf);np.minimum.at(lows,inverse,q[:,2])
    q=q[q[:,2]<=lows[inverse]+.08]
    if len(q)<20 or np.min(np.ptp(q[:,:2],axis=0))<.5:return None
    xy=q[:,:2]-base[:2];mat=np.column_stack([xy,np.ones(len(q))])
    seed=np.linalg.lstsq(mat,q[:,2],rcond=None)[0]
    fit=least_squares(lambda x:mat@x-q[:,2],seed,loss='soft_l1',f_scale=.04)
    errors=abs(mat@fit.x-q[:,2]);good=errors<=cfg['ground_max_error']
    if good.sum()<20 or good.mean()<.6 or np.linalg.norm(fit.x[:2])>.7:return None
    if np.linalg.norm(q[good,:2].mean(axis=0)-base[:2])>.8:return None
    return dict(point=base[:2].tolist(),coefficients=fit.x.tolist(),height_m=float(fit.x[2]),
                support_points=int(good.sum()),median_error_m=float(np.median(errors[good])))



def collect_tracks(p,n,mask,cfg,rng):
    zkeys=np.floor(p[:,2]/cfg['slice_height']).astype(int)
    order=np.argsort(zkeys);keys,counts=np.unique(zkeys,return_counts=True)
    starts=np.r_[0,np.cumsum(counts)];slices=[];slice_count=0
    for i in range(len(keys)):
        ids=order[starts[i]:starts[i+1]];ids=ids[mask[ids]];candidates=[]
        for group in groups(p[ids,:2],cfg['cluster_tolerance']):
            if len(group)<cfg['slice_min_points']:continue
            q=ids[group]
            while len(q)>=cfg['slice_min_points']:
                fitted=slice_circle(p[q],n[q],cfg,rng)
                if fitted is None:break
                candidates.append(fitted)
                distance=np.linalg.norm(p[q,:2]-fitted['center'][:2],axis=1)
                q=q[abs(distance-fitted['radius'])>cfg['max_fit_error']*1.5]
        slices.append(candidates);slice_count+=len(candidates)
    return associate_slices(slices,cfg),slice_count


def duplicate_tree(fits,existing,cfg):
    lo,hi=fits[0]['a'][2],fits[-1]['b'][2]
    for other in existing:
        low=max(lo,other[0]['a'][2]);high=min(hi,other[-1]['b'][2])
        if high-low<min(.8,.5*min(hi-lo,other[-1]['b'][2]-other[0]['a'][2])):continue
        matched=[]
        for z in np.linspace(low,high,7):
            left=min(fits,key=lambda c:max(c['a'][2]-z,0,z-c['b'][2]))
            right=min(other,key=lambda c:max(c['a'][2]-z,0,z-c['b'][2]))
            def xy(c):
                a=np.asarray(c['a']);b=np.asarray(c['b'])
                return (a+(b-a)*(z-a[2])/(b[2]-a[2]))[:2]
            r1=left['fit']['radius'];r2=right['radius']
            matched.append(np.linalg.norm(xy(left)-xy(right))<max(.07,.4*min(r1,r2))
                           and abs(r1-r2)<max(.04,.25*max(r1,r2)))
        if np.mean(matched)>=.7:return other[0]['tree_id']
    return None


def extract_trunks(points,cfg):
    from collections import Counter
    _,idx=np.unique(np.floor(points/cfg['voxel']).astype(np.int64),axis=0,return_index=True)
    p=points[np.sort(idx)]
    n,curvature,dense,spacing=surface_features(p,cfg,return_spacing=True)
    side=dense&(curvature<=cfg['normal_curvature'])&(abs(n[:,2])<=cfg['normal_z'])
    sparse=dict(cfg,slice_height=cfg['sparse_slice_height'],
                slice_min_points=cfg['sparse_slice_min_points'],min_angular_bins=cfg['sparse_angular_bins'],
                min_height=cfg['sparse_min_height'],min_points=max(30,int(cfg['min_points']*.75)),
                max_slice_gap=cfg['sparse_max_slice_gap'])
    sparse_side=dense&(abs(n[:,2])<=cfg['normal_z'])&(
        (curvature<=cfg['normal_curvature'])|
        ((spacing>cfg['voxel']*cfg['sparse_spacing_factor'])&(curvature<=cfg['sparse_normal_curvature'])))
    passes=[];all_tracks=[]
    for label,fit_cfg in [('standard',cfg),('sparse',sparse)]:
        mask=side if label=='standard' else sparse_side
        tracks,count=collect_tracks(p,n,mask,fit_cfg,np.random.default_rng(20261013))
        passes.append(dict(name=label,slice_height_m=fit_cfg['slice_height'],slice_candidates=count,tracks=len(tracks)))
        all_tracks.extend((t,fit_cfg,label) for t in tracks)
    # Try the longest observations first so a coarse rescue does not duplicate
    # an already identified tree. Every suppressed track is still accounted for.
    all_tracks.sort(key=lambda item:np.ptp([s['center'][2] for s in item[0]]),reverse=True)
    tree=cKDTree(p[:,:2]);colliders=[];rejected=[];accepted=[];diagnostics=[]
    ground_counts=Counter()
    for track,fit_cfg,label in all_tracks:
        centers=np.array([s['center'] for s in track]);height=float(np.ptp(centers[:,2]))
        record=dict(seed=centers[0].tolist(),top=centers[-1].tolist(),height_m=height,
                    slices=len(track),extraction_pass=label)
        def reject(reason,**details):
            item=dict(record,reason=reason,**details);rejected.append(item)
            diagnostics.append(dict(item,status='rejected'))
        if height<fit_cfg['min_height'] or len(track)<3:
            reject('insufficient_height_or_slices');continue
        bins=np.floor(centers[:,2]/fit_cfg['slice_height']).astype(int)
        occupancy=len(track)/(bins.max()-bins.min()+1)
        if occupancy<cfg['min_track_occupancy']:
            reject('insufficient_height_occupancy',height_occupancy=float(occupancy));continue
        cloud=np.vstack([s['points'] for s in track]);normals=np.vstack([s['normals'] for s in track])
        coef=np.polyfit(centers[:,2],centers[:,:2],1);slope=coef[0]
        if np.linalg.norm(slope)>np.tan(np.deg2rad(cfg['max_tilt_deg'])):
            reject('excessive_axis_tilt');continue
        radius=np.median([s['radius'] for s in track])
        overall=cylinder_fit(cloud,normals,centers.mean(axis=0),slope,radius,fit_cfg)
        line=np.column_stack([np.polyval(coef[:,j],centers[:,2]) for j in range(2)])
        bend=np.percentile(np.linalg.norm(centers[:,:2]-line,axis=1),90)
        curved=bend>cfg['bend_threshold'] or overall is None
        lo,hi=np.quantile(cloud[:,2],[.01,.99]);fits=[]
        boundaries=np.linspace(lo,hi,max(1,int(np.ceil((hi-lo)/cfg['segment_length'])))+1) if curved else np.array([lo,hi])
        # With sparse data a segment must contain enough measured layers. Grow
        # its fit window rather than fitting tiny fragments from two points.
        for low,high in zip(boundaries[:-1],boundaries[1:]):
            if curved:
                point_pad=.15 if label=='standard' else max(.3,.6*fit_cfg['slice_height'])
                center_pad=.3 if label=='standard' else point_pad
                use=(cloud[:,2]>=low-point_pad)&(cloud[:,2]<=high+point_pad)
                cs=centers[(centers[:,2]>=low-center_pad)&(centers[:,2]<=high+center_pad)]
                if len(cs)<2:fits=[];break
                local_slope=np.polyfit(cs[:,2],cs[:,:2],1)[0]
                f=cylinder_fit(cloud[use],normals[use],cs.mean(axis=0),local_slope,radius,fit_cfg)
            else:f=overall
            if f is None:fits=[];break
            fits.append(dict(fit=f,a=axis_at(f,low),b=axis_at(f,high)))
        if not fits:
            reject('cylinder_fit_failed');continue
        for left,right in zip(fits[:-1],fits[1:]):
            joint=(left['b']+right['a'])/2;left['b']=joint;right['a']=joint
        # Terrain controls base completion, never existence of observed wood.
        first=fits[0];ground=ground_plane(p,n,curvature,first['a'],first['fit']['radius'],cfg,tree)
        ground_status='unobserved';extension=0.
        if ground is not None:
            axis=first['b']-first['a'];axis/=np.linalg.norm(axis)
            gx,gy,gz=ground['coefficients'];base=first['a'];ref=np.array(ground['point'])
            denominator=axis[2]-np.dot([gx,gy],axis[:2])
            distance=(gz+np.dot([gx,gy],base[:2]-ref)-base[2])/denominator
            if -cfg['ground_max_extension']<=distance<=cfg['max_fit_error']:
                first['a']=base+distance*axis;ground_status='anchored';extension=float(max(0,-distance))
            else:ground_status='extension_limit'
        valid=True
        for segment in fits:
            a,b=segment['a'],segment['b'];d=b-a;length=np.linalg.norm(d);direction=d/length
            v=cloud-a;t=v@direction;radial=v-t[:,None]*direction
            inside=(t>=0)&(t<=length)
            if inside.sum()<fit_cfg['min_points']:valid=False;break
            error=abs(np.linalg.norm(radial[inside],axis=1)-segment['fit']['radius'])
            if np.mean(error<=cfg['max_fit_error'])<.65:valid=False;break
            segment['final_median_error_m']=float(np.median(error))
        if not valid:
            reject('joined_segment_support_failed');continue
        duplicate=duplicate_tree(fits,accepted,cfg)
        if duplicate is not None:
            reject('duplicate_observation',matched_tree_id=duplicate);continue
        tree_id=len(accepted)+1;segments=[];ground_counts[ground_status]+=1
        for i,s in enumerate(fits):
            f=s['fit'];segments.append(dict(kind='trunk',tree_id=tree_id,segment_index=i,
                a=s['a'].tolist(),b=s['b'].tolist(),radius=f['radius'],support_points=f['support_points'],
                fit_median_error_m=s['final_median_error_m'],normal_agreement=f['normal_agreement'],
                fit_method='multiscale normal-aware oriented cylinder',ground=ground,
                ground_status=ground_status,ground_extension_m=extension if i==0 else 0.,
                observed_z_range_m=[float(lo),float(hi)],extraction_pass=label,
                height_occupancy=float(occupancy),curved=bool(curved)))
        accepted.append(segments);colliders.extend(segments)
        diagnostics.append(dict(record,status='accepted',tree_id=tree_id,segments=len(segments),
                                ground_status=ground_status,height_occupancy=float(occupancy)))
    stats=dict(downsampled_points=len(p),surface_candidates=int(side.sum()),
               slice_candidates=sum(s['slice_candidates'] for s in passes),passes=passes,
               sparse_added_surface_points=int((sparse_side&~side).sum()),
               associated_tracks=len(all_tracks),accepted_trees=len(accepted),
               grounded_trees=ground_counts['anchored'],observed_only_trees=len(accepted)-ground_counts['anchored'],
               ground_status_counts=dict(ground_counts),normal_outliers=int((~dense).sum()),
               track_rejections=dict(Counter(r['reason'] for r in rejected)),candidate_diagnostics=diagnostics)
    return colliders,rejected,p,n,curvature,dense&(curvature<=cfg['normal_curvature']),stats




def segment_distance(point,c):
    a=np.asarray(c['a']);b=np.asarray(c['b']);d=b-a
    t=np.clip(np.dot(point-a,d)/np.dot(d,d),0,1)
    return float(np.linalg.norm(point-a-t*d))


def extract_branches(p,n,surface,trunks,cfg):
    # Only surface-supported tubes connected to an accepted tree can survive.
    keep=surface.copy()
    for c in trunks:
        a=np.asarray(c['a']);d=np.asarray(c['b'])-a;length=np.linalg.norm(d);axis=d/length
        v=p-a;t=v@axis;radial=np.linalg.norm(v-t[:,None]*axis,axis=1)
        keep &= ~((t>=-.15)&(t<=length+.15)&(radial<c['radius']+.15))
    ids=np.flatnonzero(keep);q=p[ids];candidates=[]
    if not len(q):return [],dict(branch_candidates=0,rejected_detached_branches=0)
    keys=np.floor(q/cfg['branch_cell']).astype(int)
    _,inverse,counts=np.unique(keys,axis=0,return_inverse=True,return_counts=True)
    order=np.argsort(inverse);starts=np.r_[0,np.cumsum(counts)]
    for i in np.flatnonzero(counts>=20):
        local=ids[order[starts[i]:starts[i+1]]];cloud=p[local]
        center=cloud.mean(axis=0);vals,vec=np.linalg.eigh(np.cov((cloud-center).T));axis=vec[:,-1]
        if vals[-1]<6*max(vals[-2],1e-9) or abs(axis[2])>.8:continue
        t=(cloud-center)@axis;radial=np.linalg.norm(cloud-center-t[:,None]*axis,axis=1)
        if np.quantile(radial,.9)>cfg['max_branch_radius']+.06:continue
        low,high=np.quantile(t,[.02,.98])
        if high-low<.35:continue
        candidates.append(dict(a=center+low*axis,b=center+high*axis,ids=local,axis=axis,
                               radius=float(np.median(radial))))
    # Connect cells along the same axis before refitting the accumulated points.
    parent=list(range(len(candidates)))
    def root(i):
        while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
        return i
    for i,c in enumerate(candidates):
        for j in range(i):
            other=candidates[j]
            if abs(np.dot(c['axis'],other['axis']))<.94 or abs(c['radius']-other['radius'])>.06:continue
            distance=min(np.linalg.norm(x-y) for x in (c['a'],c['b']) for y in (other['a'],other['b']))
            center_gap=np.linalg.norm(np.cross((c['a']+c['b']-other['a']-other['b'])/2,c['axis']))
            if distance<=cfg['branch_join_gap'] and center_gap<max(.12,c['radius']+other['radius']+.06):parent[root(i)]=root(j)
    merged={}
    for i,c in enumerate(candidates):merged.setdefault(root(i),[]).extend(c['ids'])
    fitted=[]
    for members in merged.values():
        ids=np.unique(members);cloud=p[ids];normals=n[ids];center=cloud.mean(axis=0)
        vals,vec=np.linalg.eigh(np.cov((cloud-center).T));axis=vec[:,-1]
        if abs(axis[2])>.8:continue
        u=np.cross(axis,[0.,0,1]);u/=np.linalg.norm(u);v=np.cross(axis,u)
        basis=np.column_stack([u,v]);xy=(cloud-center)@basis
        seed=np.r_[0.,0.,np.clip(np.median(np.linalg.norm(xy,axis=1)),cfg['min_branch_radius'],cfg['max_branch_radius'])]
        fit=least_squares(lambda x:np.linalg.norm(xy-x[:2],axis=1)-x[2],seed,
                bounds=([-cfg['max_branch_radius'],-cfg['max_branch_radius'],cfg['min_branch_radius']],
                        [cfg['max_branch_radius'],cfg['max_branch_radius'],cfg['max_branch_radius']]),
                loss='soft_l1',f_scale=.02)
        center=center+basis@fit.x[:2];t=(cloud-center)@axis;radial=cloud-center-t[:,None]*axis
        dist=np.linalg.norm(radial,axis=1);error=abs(dist-fit.x[2])
        agree=abs(np.sum(normals*radial,axis=1))/np.maximum(dist,1e-9)
        good=(error<=cfg['max_fit_error'])&(agree>=cfg['normal_agreement'])
        if good.sum()<cfg['min_points'] or good.mean()<.7 or angular_bins(radial[good],axis)<cfg['min_angular_bins']:continue
        low,high=np.quantile(t[good],[.01,.99])
        if high-low<cfg['min_branch_length']:continue
        bins=np.unique(np.floor((t[good]-low)/.25).astype(int))
        if len(bins)/(int(np.ceil((high-low)/.25))+1)<.7:continue
        fitted.append(dict(kind='branch',a=(center+low*axis).tolist(),b=(center+high*axis).tolist(),
                           radius=float(fit.x[2]),support_points=int(good.sum()),
                           fit_median_error_m=float(np.median(error[good])),
                           normal_agreement=float(np.median(agree[good])),fit_method='connected normal-supported tube'))
    accepted=[];pending=fitted.copy()
    while pending:
        next_pending=[]
        for c in pending:
            attached=None
            for target in trunks+accepted:
                if any(segment_distance(np.asarray(end),target)<=target['radius']+c['radius']+cfg['attachment_gap'] for end in (c['a'],c['b'])):
                    attached=target;break
            if attached is None:next_pending.append(c)
            else:
                # Extend only the short attachment gap, along the fitted branch axis.
                ends=[np.asarray(c['a']),np.asarray(c['b'])]
                index=int(np.argmin([segment_distance(end,attached) for end in ends]))
                ta=np.asarray(attached['a']);td=np.asarray(attached['b'])-ta
                near=ta+np.clip(np.dot(ends[index]-ta,td)/np.dot(td,td),0,1)*td
                direction=ends[1]-ends[0];direction/=np.linalg.norm(direction)
                shift=np.dot(near-ends[index],direction)*direction
                if np.linalg.norm(shift)>attached['radius']+c['radius']+cfg['attachment_gap']:
                    next_pending.append(c);continue
                lateral=np.linalg.norm(near-ends[index]-shift)
                if lateral>attached['radius']+c['radius']:
                    next_pending.append(c);continue
                c['a' if index==0 else 'b']=(ends[index]+shift).tolist()
                c['attachment_extension_m']=float(np.linalg.norm(shift))
                c['tree_id']=attached['tree_id'];accepted.append(c)
        if len(next_pending)==len(pending):break
        pending=next_pending
    return accepted,dict(branch_candidates=len(candidates),merged_branch_groups=len(merged),fitted_branch_chains=len(fitted),
                         rejected_detached_branches=len(pending))

