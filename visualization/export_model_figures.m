function export_model_figures()
%EXPORT_MODEL_FIGURES Save screenshots of the Simulink model (top level, controller,
%   fuzzy subsystem, digital twin plant) in outputs/figures, for the README and the report.
root = init_project();
mdl = 'cartpole_fuzzy_twin';
load_system(fullfile(root, 'simulink', [mdl '.slx']));
figDir = fullfile(root, 'outputs', 'figures');
views = {mdl, 'simulink_top_level';
         [mdl '/Controller (Ts = 10 ms)'], 'simulink_controller';
         [mdl '/Controller (Ts = 10 ms)/Fuzzy balance Mamdani'], 'simulink_fuzzy_balance';
         [mdl '/Digital Twin (Simscape Multibody)'], 'simulink_digital_twin'};
for i = 1:size(views, 1)
    print(['-s' views{i,1}], '-dpng', '-r150', fullfile(figDir, [views{i,2} '.png']));
end
fprintf('Model screenshots saved in %s\n', figDir);
end
