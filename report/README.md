# report/ — project report (LaTeX)

`report.tex` is the full write-up: model, swing-up, fuzzy controller design (membership functions, rule table, Mamdani vs Sugeno inference, LQR-based scaling), switching logic, the Simulink digital twin, results of the four scenarios and discussion. `report.pdf` is the compiled version.

The report uses the figures in `../outputs/figures/` and includes the results table `../outputs/results/comparison_table.tex`, which is generated from the simulations. To update everything after changing the controllers:

```matlab
main_compare            % new simulations, figures, comparison.csv
export_model_figures    % Simulink screenshots
main_twin(1, true)      % animation frames/video
export_latex_table      % comparison_table.tex for the report
```

Compile (twice, for the references) with `pdflatex report.tex`, or upload the whole project folder to Overleaf and compile `report/report.tex`.
