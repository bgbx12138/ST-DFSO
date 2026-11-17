"""Export stored final-route measures. No model, optimisation or statistical reruns."""
from pathlib import Path
import argparse,csv,json
from result_data import ROUTE_FILE, load_routes, validate_outcome

SCENES=[('S1','S1','Taiwan Strait'),('S2','S2','South China Sea'),('S3','S3','Miyako Strait'),('S4','S4','Bashi Channel'),('S5','S5','Penghu region'),('S6','S6','Qingdao–Busan')]
GOALS=[('TIME','Time'),('FUEL','Energy'),('SAFETY_DENSITY','Average ship-motion cost'),('SAFETY_EXPOSURE','Cumulative exposure'),('COMPOSITE','Composite cost')]
ALGORITHMS=[('FSA_SELECTED','ST-DFSO'),('ACO','ACO'),('ASV_IAFSA','IAFSA'),('WINDOW_ILS','Window ILS')]
FIELDS=[('TIME','voyage_time_h','time_hours'),('FUEL','energy_J','fuel_j'),('SAFETY_DENSITY','mean_ship_motion_cost','risk_density'),('SAFETY_EXPOSURE','cumulative_exposure','risk_total'),('COMPOSITE','composite_cost','composite')]
def export(root,output):
 root=Path(root);output=Path(output)
 if output.exists():raise FileExistsError(output)
 data=json.loads((root/'data/results/accepted_runs.json').read_text('utf-8'))
 coordinates=load_routes(root)
 index={(r['job']['scene'],r['job']['goal'],r['job']['method'],r['job']['replicate']):r for r in data if 'MAIN' in r['job']['views']}
 allrows=[]
 for scene_i,(scene,paper,region) in enumerate(SCENES):
  for goal,goal_name in GOALS:
   for rep in [1,2,3]:
    for method,method_name in ALGORITHMS:
     r=index[scene,goal,method,rep]
     validate_outcome(root,r,coordinates)
     row={'paper_case':paper,'region':region,'optimisation_objective':goal_name,'algorithm':method_name,'paired_run':rep,'random_seed':r['job']['seed']}
     for key,field,_ in FIELDS:
      v=r['final']['five_goal_values'][key]
      row[field]=v
     row['distance_n_mile']=r['final']['distance_n_mile']
     row.update(source_scene=scene,source_objective=goal,source_algorithm=method,original_run_id=r['original_job_id'],selected_run_id=r['job']['id'],route_file=ROUTE_FILE,route_key=r['route_key'],route_sha256=r['route_sha256'])
     allrows.append(row)
 assert len(allrows)==360 and len({r['original_run_id'] for r in allrows})==360
 columns=['paper_case','region','optimisation_objective','algorithm','paired_run','random_seed','voyage_time_h','distance_n_mile','energy_J','mean_ship_motion_cost','cumulative_exposure','composite_cost','source_scene','source_objective','source_algorithm','original_run_id','selected_run_id','route_file','route_key','route_sha256']
 output.parent.mkdir(parents=True,exist_ok=True)
 with output.open('w',newline='',encoding='utf-8') as f:
  writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader();writer.writerows(allrows)
 print(json.dumps({'routes':360,'stored_outcomes_verified':360,'route_coordinates_verified':360,'output':str(output),'new_scientific_evaluations':0}))
 return allrows
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--output',type=Path,required=True);args=p.parse_args();export(args.root,args.output)
