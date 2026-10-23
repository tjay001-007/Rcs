% run_viz.m  — run this after each sim to see the trajectory on the Cesium globe
%
%   From the Command Window:  run_viz
%   Or press Run (F5) with this file open in the editor.
%
%   To also export a Google Earth KML:
%       run_viz; export_kml(recef_log)

sc = viz_satscenario(recef_log);