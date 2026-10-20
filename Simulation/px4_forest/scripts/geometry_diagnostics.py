"""Comparable observed-surface support and per-candidate extraction diagnostics."""
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree


def surface_support(points,geometry,eligible=None,tolerance=.08):
    """Union of observed points close to finite cylinder sides, not semantic recall."""
    points=np.asarray(points);tree=cKDTree(points);covered=np.zeros(len(points),dtype=bool)
    for c in geometry['colliders']:
        a=np.asarray(c['a']);b=np.asarray(c['b']);d=b-a;length=np.linalg.norm(d);axis=d/length
        ids=np.asarray(tree.query_ball_point((a+b)/2,np.hypot(length/2,c['radius']+tolerance)),dtype=int)
        v=points[ids]-a;t=v@axis;radial=np.linalg.norm(v-t[:,None]*axis,axis=1)
        good=(t>=0)&(t<=length)&(abs(radial-c['radius'])<=tolerance)
        covered[ids[good]]=True
    if eligible is not None:covered &= eligible
    return covered


def retained_baseline_trees(old,new,tolerance=.2):
    trees={}
    for c in old['colliders']:
        if c['kind']=='trunk':trees.setdefault(c['tree_id'],[]).append(c)
    matched={}
    for tid,cs in trees.items():
        q=np.vstack([np.linspace(c['a'],c['b'],9) for c in cs]);close=np.zeros(len(q),dtype=bool)
        for c in new['colliders']:
            if c['kind']!='trunk':continue
            a=np.asarray(c['a']);d=np.asarray(c['b'])-a;length=np.linalg.norm(d);axis=d/length
            v=q-a;t=v@axis
            distance=np.linalg.norm(v-t[:,None]*axis,axis=1)
            close |= (t>=-.1)&(t<=length+.1)&(distance<=tolerance)
        matched[str(tid)]=float(close.mean())
    return matched


def write_candidate_diagnostics(points,geometry,output):
    output=Path(output);records=geometry.get('candidate_diagnostics',[])
    fields=['status','reason','tree_id','matched_tree_id','extraction_pass','height_m','slices',
            'height_occupancy','ground_status','seed_x','seed_y','seed_z','top_x','top_y','top_z']
    with (output/'candidate_diagnostics.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for record in records:
            row={k:record.get(k,'') for k in fields}
            for name in ('seed','top'):
                for axis,value in zip('xyz',record.get(name,[])):row[name+'_'+axis]=value
            writer.writerow(row)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    fig,axes=plt.subplots(1,2,figsize=(14,6));sample=points[::max(1,int(np.ceil(len(points)/100000)))]
    axes[0].scatter(sample[:,0],sample[:,1],c='gray',s=.2,alpha=.22)
    labelled=set()
    for c in geometry['colliders']:
        if c['kind']!='trunk':continue
        a,b=np.array(c['a']),np.array(c['b']);grounded=c.get('ground_status')=='anchored'
        color='saddlebrown' if grounded else 'royalblue'
        label='Ground anchored' if grounded else 'Observed segment; base incomplete'
        axes[0].plot([a[0],b[0]],[a[1],b[1]],color=color,lw=1,label=label if label not in labelled else None)
        labelled.add(label);axes[0].add_patch(Circle(a[:2],c['radius'],fill=False,color=color,lw=.7))
        if c.get('segment_index',0)==0:axes[0].text(a[0],a[1],str(c['tree_id']),fontsize=5)
    axes[0].set(xlabel='local x (m)',ylabel='local y (m)',title='Accepted trunk hypotheses against full PCD')
    axes[0].set_aspect('equal');axes[0].legend(fontsize=8);axes[0].grid(alpha=.2)
    reasons=geometry['extraction_statistics'].get('track_rejections',{})
    names=list(reasons);values=[reasons[name] for name in names]
    axes[1].barh(np.arange(len(names)),values,color='slategray')
    axes[1].set_yticks(np.arange(len(names)));axes[1].set_yticklabels(names,fontsize=8)
    for i,value in enumerate(values):axes[1].text(value,i,' '+str(value),va='center',fontsize=8)
    axes[1].set(xlabel='Tracks (both scales; includes duplicates)',title='Every rejected track has a recorded reason')
    counts=geometry['counts'];fig.suptitle('%d hypotheses: %d anchored, %d observed-only. Counts are not labeled tree recall.'%
                         (counts['trees'],counts['grounded_trees'],counts['observed_only_trees']))
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(output/'candidate_map.png',dpi=150);plt.close(fig)
    summary=dict(counts=geometry['counts'],statistics=geometry['extraction_statistics'],
                 median_segment_radial_error_m=float(np.median([c['fit_median_error_m'] for c in geometry['colliders']])) if geometry['colliders'] else None,
                 note='Observed support residuals and hypothesis counts do not measure labeled precision/recall.')
    (output/'extraction_summary.json').write_text(json.dumps(summary,indent=2)+'\n')



def compare_geometry(points,baseline,updated,output,tolerance=.08):
    if baseline['local_map_sha256']!=updated['local_map_sha256']:
        raise ValueError('Comparison requires the same local PCD hash')
    from trunk_extraction import surface_features,DEFAULTS
    cfg=dict(DEFAULTS);cfg.update(baseline['parameters'])
    _,ids=np.unique(np.floor(points/cfg['voxel']).astype(np.int64),axis=0,return_index=True)
    common=points[np.sort(ids)];normals,curvature,dense=surface_features(common,cfg)
    eligible=dense&(curvature<=cfg['normal_curvature'])&(abs(normals[:,2])<=cfg['normal_z'])
    old_mask=surface_support(common,baseline,eligible,tolerance)
    new_mask=surface_support(common,updated,eligible,tolerance)
    matches=retained_baseline_trees(baseline,updated)
    denominator=int(eligible.sum())
    report=dict(baseline_counts=baseline['counts'],updated_counts=updated['counts'],
        input_map_sha256=updated['local_map_sha256'],common_candidate_points=denominator,
        surface_tolerance_m=tolerance,
        baseline_supported_points=int(old_mask.sum()),updated_supported_points=int(new_mask.sum()),
        baseline_support_fraction=float(old_mask.sum()/max(1,denominator)),
        updated_support_fraction=float(new_mask.sum()/max(1,denominator)),
        supported_point_gain=int((new_mask&~old_mask).sum()),
        baseline_supported_points_not_recovered=int((old_mask&~new_mask).sum()),
        baseline_tree_axis_coverage=matches,
        baseline_trees_at_least_half_axis_retained=sum(x>=.5 for x in matches.values()),
        median_segment_radial_error_m=dict(
            baseline=float(np.median([c['fit_median_error_m'] for c in baseline['colliders']])),
            updated=float(np.median([c['fit_median_error_m'] for c in updated['colliders']]))),
        limits='Fixed-mask observed vertical-surface support proxy only; not labeled tree precision/recall. No unseen surface correctness claim.')
    output=Path(output);(output/'coverage_comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,6),sharex=True,sharey=True)
    sample=points[::max(1,int(np.ceil(len(points)/100000)))]
    for ax,g,label in zip(axes,[baseline,updated],['Baseline','Updated']):
        ax.scatter(sample[:,0],sample[:,1],s=.2,c='gray',alpha=.2)
        for c in g['colliders']:
            a,b=np.array(c['a']),np.array(c['b'])
            color='saddlebrown' if c.get('ground_status','anchored')=='anchored' else 'royalblue'
            ax.plot([a[0],b[0]],[a[1],b[1]],c=color,lw=1)
            if c.get('segment_index',0)==0:ax.scatter(a[0],a[1],s=8,facecolors='none',edgecolors=color)
        ax.set(title='%s: %d trunk hypotheses'%(label,g['counts']['trees']),xlabel='local x (m)',ylabel='local y (m)')
        ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.suptitle('Same PCD / same coordinates. Brown: anchored; blue: measured segment with incomplete base.')
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(output/'coverage_comparison.png',dpi=150);plt.close(fig)
    return report

