% case39_no_ground_admittance_3panel_fullband_RHPred.m
% -------------------------------------------------------------------------
% IEEE 39-bus no-ground-admittance case study for the paper.
% This script matches Lemma 1: no static load admittance, no bus GS/BS shunt,
% and no artificial grounding floor in the dynamic network. Converter-control
% parameters are kept consistent with the original script; constant-power
% loads are absorbed into local converter operating-point injections.
% The script keeps the original case-study logic, but uses the worst bus
% (largest g_min) as the representative bus and uses a 3-panel figure:
% (a) local gains, (b) full-frequency passivity index, (c) rightmost closed-loop eigenvalues.
% It also prints isolated-network and closed-loop eigenvalue diagnostics.
% -------------------------------------------------------------------------

clear; clc; close all;
define_constants;

%% ------------------------------ Style (IEEE Trans) ---------------------------
set_plot_defaults_trans_style();

%% ------------------------------ User configuration ----------------------------
cfg.case_name   = 'case39';
cfg.f0          = 60;                 % Hz
cfg.w0          = 2*pi*cfg.f0;

% Converters placed at ALL buses (39 converters in case39)
cfg.use_gen_buses_only = false;

% Important modeling choice for the no-ground-admittance test:
% The dynamic network contains only bus shunt capacitances and RL line dynamics.
% Static load admittances, bus GS/BS shunts, and artificial grounding floors are removed
% from the dynamic network. Constant-power loads from the power flow are absorbed into
% the local converter operating points as negative net injections.
cfg.all_bus_converter_mode = 'net_bus_injection_no_ground';
% Stress profile (weak grid): increase x, reduce r
cfg.stress.x_scale    = 1.5;
cfg.stress.r_scale    = 0.7;
cfg.stress.load_scale = 1.0;

% Frequency grids
cfg.f_grid_Hz  = logspace(-4, 4, 2800);        % 1e-4 ... 1e4 Hz (two-pass refinement used for g_min)
cfg.w_grid     = 2*pi*cfg.f_grid_Hz;           % rad/s for freqresp

cfg.f_net_diag = logspace(-2, 3, 220);         % 1e-2 ... 1e3 Hz
cfg.w_net_diag = 2*pi*cfg.f_net_diag;

% Network preprocessing for the no-ground-admittance model:
% 1) remove off-nominal taps / phase shifts so the dynamic network matches
%    the simple incidence-matrix pi-line model used in the proof;
% 2) remove bus GS/BS shunts and do NOT add any grounding conductance floor.
%    Line charging BR_B is retained and lumped into the dynamic capacitance C_b.
cfg.paper_match.remove_taps_and_shifts = true;
cfg.paper_match.remove_bus_shunts      = true;
cfg.paper_match.gs_floor_pu            = 0;      % no artificial grounding conductance

% Virtual shunt margin
cfg.g_margin_rel = 0.02;       % +2% base margin above sampled g_min
cfg.g_margin_abs = 1e-4;       % +1e-4 pu
cfg.eta_margin_abs = 1e-7;     % sampled nonnegative-margin target for eta(omega)
cfg.enforce_passivated_stability = true;
cfg.stable_margin_target = 1e-5;
cfg.g_scale_candidates = [1.0 1.01 1.02 1.05 1.10 1.20 1.50 2 3];

% Network numerical options
netopt.g_eps                    = 0;      % must remain zero for the no-ground-admittance test
netopt.c_min                    = 1e-6;   % small dynamic capacitance floor to keep C_b positive
netopt.use_static_load_admittance = false; % no Y_L in the network state matrix
netopt.assert_positive_G        = false;  % G is intentionally zero when Y_L is removed

% Converter base parameters (pu on system base)
prm.Xf   = 0.15;               % filter reactance at w0 (pu)
prm.rf   = 0.01;               % filter resistance (pu)

% Inner current loop tuning (2nd-order target)
prm.w_ci   = 2*pi*300;         % rad/s (current-loop bandwidth ~300 Hz)
prm.zeta_i = 1.0;              % damping ratio

% Voltage loop candidates (broadened slightly for the all-bus case)
tune.Kpv_list = [0.05 0.10 0.20 0.30 0.50 0.80 1.0 1.5 2.0 3.0 5.0];
tune.Kiv_list = [1e-3 2e-3 5e-3 0.01 0.02 0.05 0.10 0.20 0.50 1.0 2.0 5.0 10.0];
tune.stab_margin = 2e-3;       % want maxRe(A_conv) <= -2e-3
tune.auto_expand   = true;      % if needed, automatically expand the search grid
tune.allow_dp_relax = false;     % keep the paper droop gain unchanged by default

% Droop + power filters (paper model)
prm.dp    = 5.0;               % rad/s per pu active power
prm.dq    = 0.01;              % pu voltage per pu reactive power
prm.tau_p = 0.05;              % s
prm.tau_q = 0.05;              % s

% If empty => choose worst converter (max gmin) for display
cfg.force_bus_display = [];   % [] means choose automatically by display_bus_policy
cfg.display_bus_policy = 'worst_overall';  % use the largest g_min bus as representative after removing grounding

% Heterogeneous all-bus support profile:
%   - generator buses keep the paper's GFM droop settings;
%   - non-generator buses use milder voltage-support GFM parameters. In the
%     no-ground model their operating points may be net loads, because
%     constant-power loads are absorbed into local injections.
cfg.support_profile.enable          = true;
cfg.support_profile.dp_non_gen      = 0.25;
cfg.support_profile.dq_non_gen      = 0.05;
cfg.support_profile.tau_p_non_gen   = 0.20;
cfg.support_profile.tau_q_non_gen   = 0.20;

% Optional time-domain disturbance settings retained for diagnostics.
% The publication figure in this version does not plot a time-domain panel.
cfg.time_response.axis           = 'q';
cfg.time_response.id_amp         = 1e-3;
cfg.time_response.id_f_Hz        = 3.0;
cfg.time_response.burst_duration = 0.8;
cfg.time_response.t_end          = 1.2;
cfg.time_response.inset_t_end    = 0.9;

% Innovation-focused figure controls
cfg.fig3b_inset_peak_span = 3.0;      % inset band = [f*/span, f*span]
cfg.fig3c_mark_timescales = true;
cfg.fig3c_relerr_target   = 0.10;     % retained for optional diagnostics
cfg.fig3c_miderr_target = 0.25;      % empirical onset threshold for the displayed MF regime

% Export options for publication-ready figures
cfg.export.enable     = true;
cfg.export.basename   = sprintf('%s_no_ground_admittance_3panel_fullband_RHPred', cfg.case_name);
cfg.export.resolution = 600;

% Save a small numerical summary for discussion with the supervisor.
cfg.write_summary_txt = true;
cfg.summary_txt = sprintf('%s_no_ground_admittance_numeric_summary.txt', cfg.case_name);

rng(1);
fprintf('=== Demo start: %s, f0=%.1f Hz (ALL-BUS converter case) ===\n', cfg.case_name, cfg.f0);

%% ------------------------------ 1) Power flow --------------------------------
mpopt = mpoption('verbose',0,'out.all',0, ...
                 'pf.alg','NR','pf.tol',1e-10,'pf.nr.max_it',50, ...
                 'pf.enforce_q_lims',0);

[res, stress_used] = run_pf_stressed_safe(cfg.case_name, cfg.stress, mpopt, cfg.paper_match);
baseMVA = res.baseMVA;

fprintf('  Power flow converged (baseMVA=%.1f, x_scale=%.3g, r_scale=%.3g, load_scale_used=%.3g).\n', ...
    baseMVA, stress_used.x_scale, stress_used.r_scale, stress_used.load_scale_used);
if stress_used.remove_taps_and_shifts
    fprintf('  Paper matching: off-nominal taps/phase shifts removed in PF and dynamic model.\n');
end
if isfield(stress_used,'remove_bus_shunts') && stress_used.remove_bus_shunts
    fprintf('  No-ground-admittance setting: bus GS/BS shunts removed; no GS floor is applied.\n');
else
    fprintf('  Paper matching: GS floor = %.3g pu applied at every bus in the PF model.\n', stress_used.gs_floor_pu);
end

%% ------------------------------ 2) Choose converter buses ---------------------
[conv_bus_ids, conv_bus_idx, Ssel] = select_converter_buses(res, cfg.use_gen_buses_only);
Nc = numel(conv_bus_idx);
gen_bus_ids = unique(res.gen(:, GEN_BUS));
nZeroEq = sum(~ismember(conv_bus_ids, gen_bus_ids));

fprintf('  Converters: Nc=%d (%s buses).\n', Nc, ternary(cfg.use_gen_buses_only,'GEN','ALL'));
fprintf('  Non-generator buses: %d. In the no-ground model their loads are absorbed into local net injections.\n', nZeroEq);

%% ------------------------------ 3) Build operating points ---------------------
op = build_converter_operating_points(res, conv_bus_idx, cfg.all_bus_converter_mode);

%% ------------------------------ 4) Build network (dq state-space) -------------
net = build_network_ss_dq(res, cfg.w0, netopt);
fprintf('  Network: N=%d buses, M=%d in-service branches.\n', net.N, net.M);
fprintf('  Dynamic network model: Y_L disabled. Only C_b and RL line dynamics are retained.\n');

% Network eigenvalue diagnostic for the isolated network subsystem
Anet = [net.A_vv, net.A_vi;
        net.A_iv, net.A_ii];
eig_net = eig(full(Anet));
print_network_eigen_summary(eig_net, cfg.w0);

% Network passivity diagnostic (PORT admittance via Kron reduction).
% For the no-ground-admittance model this is expected to be positive semidefinite,
% not strictly positive definite; a near-zero minimum at f0 reflects the common mode.
w_net_diag_ext = unique([cfg.w_net_diag(:); cfg.w0]);
[lam_net, lam_net_min, f_at_min] = network_port_passivity_curve(net, cfg.w0, w_net_diag_ext, conv_bus_idx);
fprintf('  Network port min eig(sym(Y_net,port(jw))) on diag grid: %.3e (at f=%.6g Hz)\n', lam_net_min, f_at_min);
if lam_net_min < -1e-8
    warning('Network passivity diagnostic is negative beyond numerical tolerance. Check signs and model consistency.');
end

%% ------------------------------ 5) Converter gains ----------------------------
% Current loop gains from bandwidth & damping (pu-consistent)
Lf = prm.Xf / cfg.w0;
prm.Kpc = 2*prm.zeta_i*prm.w_ci*Lf - prm.rf;
prm.Kic = (prm.w_ci^2)*Lf;
if prm.Kpc <= 0
    warning('Computed Kpc<=0; forcing a small positive Kpc. Consider increasing w_ci.');
    prm.Kpc = 0.05;
end

% Voltage loop tuned to make every local converter model internally stable
[prm_tuned, conv0, worstReA] = tune_voltage_loop_all(op, prm, cfg.w0, tune, cfg);
fprintf('  Selected voltage loop: Kpv=%.3g, Kiv=%.3g | worst maxReAconv=%.3e\n', ...
    prm_tuned.Kpv, prm_tuned.Kiv, worstReA);
fprintf('  Non-generator support profile: dp=%.3g, dq=%.3g, tau_p=%.3g s, tau_q=%.3g s\n', ...
    cfg.support_profile.dp_non_gen, cfg.support_profile.dq_non_gen, cfg.support_profile.tau_p_non_gen, cfg.support_profile.tau_q_non_gen);

%% ------------------------------ 6) Assemble global system (no shunt) ----------
g0 = zeros(Nc,1);
Acl0 = assemble_global_A(net, conv0, Ssel, g0);
eig0 = eig(full(Acl0));
maxRe0 = max(real(eig0));
fprintf('  Global stability (no virtual shunt): maxRe(eigs)=%.4e\n', maxRe0);
print_closed_loop_eigen_summary('Closed loop without virtual shunt', eig0, 12);
if maxRe0 <= 0
    warning(['The no-shunt case is already stable with the current stress/tuning. ' ...
             'If you need an unstable baseline, first try cfg.stress.x_scale = 1.8 ' ...
             'or prm.dp = 6.0.']);
end

%% ------------------------------ 7) Compute decentralized gmin_i ---------------
gmin  = zeros(Nc,1);
g0_LF = zeros(Nc,1);
eta_min_no_shunt  = zeros(Nc,1);
eta_min_with_base = zeros(Nc,1);
peak_f_gmin = zeros(Nc,1);

for k = 1:Nc
    sysY = conv0(k).sysY_dev;                % Δv -> Δi_dev
    g0_LF(k) = g0_from_DC_limit(op(k), conv0(k).par.dq);
    [gmin(k), peak_f_gmin(k)] = compute_gmin_from_ss_refined(sysY, cfg.w_grid, g0_LF(k));

    g_base_k = (1+cfg.g_margin_rel)*gmin(k) + cfg.g_margin_abs;
    eta_min_no_shunt(k)  = sampled_eta_min(sysY, cfg.w_grid, 0);
    eta_min_with_base(k) = sampled_eta_min(sysY, cfg.w_grid, g_base_k);

    if eta_min_with_base(k) < cfg.eta_margin_abs
        g_base_k = g_base_k + (cfg.eta_margin_abs - eta_min_with_base(k));
        eta_min_with_base(k) = sampled_eta_min(sysY, cfg.w_grid, g_base_k);
    end
    gmin(k) = max(gmin(k), g_base_k/(1+cfg.g_margin_rel));  % keep bookkeeping conservative

    fprintf('    Conv@bus %d: P0=%.4g pu, Q0=%.4g pu, gmin=%.4g, g0(LF,th)=%.4g, peak(f)=%.4g Hz, eta_min(no shunt)=%.4g, eta_min(base g)=%.4g\n', ...
        op(k).bus_id, op(k).P0, op(k).Q0, gmin(k), g0_LF(k), peak_f_gmin(k), eta_min_no_shunt(k), eta_min_with_base(k));
end

g_vec_base = (1+cfg.g_margin_rel)*gmin + cfg.g_margin_abs;
for k = 1:Nc
    eta_k = sampled_eta_min(conv0(k).sysY_dev, cfg.w_grid, g_vec_base(k));
    if eta_k < cfg.eta_margin_abs
        g_vec_base(k) = g_vec_base(k) + (cfg.eta_margin_abs - eta_k);
    end
end
g_vec = g_vec_base;

% Choose buses for summary and display
[~, kWorstAll] = max(gmin);
genMask = [op.is_gen_bus];
if any(genMask)
    idxGen = find(genMask);
    [~, ixGen] = max(gmin(idxGen));
    kWorstGen = idxGen(ixGen);
else
    kWorstGen = kWorstAll;
end

if isempty(cfg.force_bus_display)
    switch lower(cfg.display_bus_policy)
        case 'worst_generator'
            kDisplay = kWorstGen;
        case 'worst_overall'
            kDisplay = kWorstAll;
        otherwise
            error('Unknown display-bus policy: %s', cfg.display_bus_policy);
    end
else
    kDisplay = find([op.bus_id] == cfg.force_bus_display, 1);
    if isempty(kDisplay), error('force_bus_display bus not found among converter buses.'); end
end

g_gap = gmin - g0_LF;
[~, kGap] = max(g_gap);

fprintf('\n  Worst overall converter @ bus %d: gmin=%.4g -> apply g=%.4g\n', ...
    op(kWorstAll).bus_id, gmin(kWorstAll), g_vec(kWorstAll));
fprintf('  Worst generator-bus converter @ bus %d: gmin=%.4g -> apply g=%.4g\n', ...
    op(kWorstGen).bus_id, gmin(kWorstGen), g_vec(kWorstGen));
fprintf('  Fig. 3 display bus @ %d (policy: %s).\n', op(kDisplay).bus_id, cfg.display_bus_policy);
[~, rankOrderGmin] = sort(gmin, 'descend');
bus31_idx = find([op.bus_id] == 31, 1);
if ~isempty(bus31_idx)
    bus31_rank = find(rankOrderGmin == bus31_idx, 1);
    fprintf('  Bus 31 diagnostic: gmin=%.4g, applied g=%.4g, rank by gmin=%d/%d.\n', ...
        gmin(bus31_idx), g_vec(bus31_idx), bus31_rank, Nc);
end
fprintf('  Largest finite-frequency excess at bus %d: gmin-g0 = %.4g\n', ...
    op(kGap).bus_id, g_gap(kGap));

%% ------------------------------ 8) Assemble global system (with shunt) --------
[g_vec, g_scale_used, Acl1, eig1, maxRe1] = enforce_passivated_stability( ...
    net, conv0, Ssel, g_vec, cfg);

fprintf('  Global stability (with virtual shunt): maxRe(eigs)=%.4e\n', maxRe1);
print_closed_loop_eigen_summary('Closed loop with virtual shunt', eig1, 12);
if g_scale_used > 1
    warning('Applied virtual shunts were uniformly scaled by %.3g to recover a strictly stable passivated case.', g_scale_used);
end
if maxRe1 >= -cfg.stable_margin_target
    warning('The passivated all-bus case is only marginally stable on the computed model. Consider increasing stress or re-checking the local tuning.');
end
fprintf('  Closed-loop spectral margins: no shunt %.4e, with shunt %.4e\n', maxRe0, maxRe1);

if isfield(cfg,'write_summary_txt') && cfg.write_summary_txt
    write_numeric_summary(cfg.summary_txt, cfg, eig_net, eig0, eig1, op, gmin, g0_LF, g_vec, kDisplay);
end

eta_min_with_applied = zeros(Nc,1);
for k = 1:Nc
    eta_min_with_applied(k) = sampled_eta_min(conv0(k).sysY_dev, cfg.w_grid, g_vec(k));
end

%% ------------------------------ 9) Data for figures ---------------------------
% 9.1 Local passivity quantities at the representative bus
[lambdaMax_w, fHz] = lambda_max_sym_vs_w(conv0(kDisplay).sysY_dev, cfg.w_grid);
% Force column vectors to avoid row/column implicit-expansion bugs in plotting masks.
fHz = fHz(:);
lambdaMax_w = lambdaMax_w(:);
eta_no_g   = -lambdaMax_w;                    % η(ω) for Y_c=-Y_dev
eta_with_g = g_vec(kDisplay) - lambdaMax_w;   % η(ω) for Y_c + gI
[eta_no_g_min, idx_eta_min] = min(eta_no_g);
f_eta_min = fHz(idx_eta_min);                 % finite-frequency worst shortfall
f_lf = max(1/(2*pi*conv0(kDisplay).par.tau_p), 1/(2*pi*conv0(kDisplay).par.tau_q));

% 9.2 No time-domain panel is plotted in this version. The previous Fig. 3(d)
% is removed to leave more space for the closed-loop eigenvalue comparison.


%% ------------------------------ 10) COMBINED FIGURE: case-study summary ---------
% One three-panel figure for the paper:
%   (a) locally computed passivation gains
%   (b) full-frequency passivity index at the representative/worst bus
%   (c) rightmost eigenvalues of the full linearized closed loop, spanning the lower row
% Styled for a restrained IEEE Transactions look and saved automatically.

C = trans_colors();

fig = create_fig_inch('Case-study summary (IEEE Trans style)', 7.10, 5.35);
tl  = tiledlayout(fig, 2, 2, 'TileSpacing','compact', 'Padding','compact');


% (a) Local passivation gains
ax = nexttile(tl,1);
bus_ids = [op.bus_id];
barColors = repmat(C.barfill, Nc, 1);
barColors(kDisplay,:) = C.barfill_hi;

hb = bar(ax, bus_ids, g_vec, 0.74, ...
    'FaceColor','flat', 'EdgeColor', C.baredge, 'LineWidth',0.70);
hb.CData = barColors;
hold(ax,'on');
bar(ax, bus_ids(kDisplay), g_vec(kDisplay), 0.74, ...
    'FaceColor','none', 'EdgeColor', C.blue, 'LineWidth',1.00, ...
    'HandleVisibility','off');

h1 = plot(ax, bus_ids, gmin, '-o', ...
    'Color', C.dark, 'MarkerSize',4.2, 'LineWidth',1.20, 'MarkerFaceColor','w');
h2 = plot(ax, bus_ids, g0_LF, '--s', ...
    'Color', C.midgray, 'MarkerSize',4.0, 'LineWidth',1.05, 'MarkerFaceColor','w');

apply_axes_trans(ax);
xlabel(ax, 'Bus ID');
ylabel(ax, '$g_i$ (p.u.)');
add_panel_label(ax, '(a)');
leg = legend(ax, [hb h1 h2], ...
    {'Applied $g_i$', 'Computed $g_{\min,i}$', 'Low-frequency asymptote $g_{0,i}$'}, ...
    'Location','northwest');
style_legend(leg);
if numel(op) > 20
    xticks(ax, [op(1:2:end).bus_id]);
end
ymax_a = max([g_vec(:); gmin(:); g0_LF(:)]);
if ~isfinite(ymax_a) || ymax_a <= 0, ymax_a = 1; end
xlim(ax, [min(bus_ids) - 0.8, max(bus_ids) + 0.8]);
ylim(ax, [0, 1.12*ymax_a]);
y_label_bus = min(1.07*ymax_a, max(g_vec(kDisplay), gmin(kDisplay)) + 0.04*ymax_a);
text(ax, bus_ids(kDisplay) + 0.15, y_label_bus, ...
    sprintf('\\textbf{bus %d}', bus_ids(kDisplay)), ...
    'Color', C.blue, 'HorizontalAlignment','left', 'VerticalAlignment','bottom');

% (b) Full-frequency passivity index at the representative bus
% The main axis shows the full frequency band used for computing g_min
% (from cfg.f_grid_Hz(1) to cfg.f_grid_Hz(end)). A small inset is kept only
% to make the near-DC/low-frequency shortfall visible without hiding the
% high-frequency behavior.
ax = nexttile(tl,2);

idx_plot_b = isfinite(fHz(:)) & isfinite(eta_no_g(:)) & isfinite(eta_with_g(:));
if nnz(idx_plot_b) < 10
    error('Not enough finite frequency-response samples for Fig. 3(b).');
end

h1 = semilogx(ax, fHz(idx_plot_b), eta_no_g(idx_plot_b), '-',  ...
    'Color', C.dark, 'LineWidth',1.20); hold(ax,'on');
h2 = semilogx(ax, fHz(idx_plot_b), eta_with_g(idx_plot_b), '--', ...
    'Color', C.blue, 'LineWidth',1.25);
yline(ax, 0, '-', 'Color', C.gray, 'LineWidth',0.85, 'HandleVisibility','off');
yline(ax, -g0_LF(kDisplay), ':', 'Color', C.midgray, 'LineWidth',0.90, ...
    'HandleVisibility','off');
yline(ax, g_vec(kDisplay)-g0_LF(kDisplay), ':', 'Color', C.blue, 'LineWidth',0.80, ...
    'HandleVisibility','off');
xline(ax, f_eta_min, ':', 'Color', C.midgray, 'LineWidth',0.90, 'HandleVisibility','off');
xline(ax, f_lf, '--', 'Color', C.gray, 'LineWidth',0.75, 'HandleVisibility','off');
plot(ax, f_eta_min, eta_no_g_min, 'o', ...
    'Color', C.dark, 'MarkerSize',5.0, 'LineWidth',1.00, 'MarkerFaceColor','w', ...
    'HandleVisibility','off');

apply_axes_trans(ax);
xlim(ax, [min(fHz(idx_plot_b)), max(fHz(idx_plot_b))]);
xlabel(ax, 'Frequency (Hz)');
ylabel(ax, '$\eta_i(\omega)$ (p.u.)');
add_panel_label(ax, '(b)');
leg = legend(ax, [h1 h2], {'Without virtual shunt','With virtual shunt'}, 'Location','northeast');
style_legend(leg);

% Use full-band values for the y-axis. This keeps the high-frequency behavior
% visible. The inset below provides the magnified low-frequency view.
vals_eta = [eta_no_g(idx_plot_b); eta_with_g(idx_plot_b); -g0_LF(kDisplay); ...
            g_vec(kDisplay)-g0_LF(kDisplay); 0];
vals_eta = vals_eta(isfinite(vals_eta));
if isempty(vals_eta)
    ylo = -1; yhi = 1;
else
    ylo = min(vals_eta); yhi = max(vals_eta);
    yrng = max(yhi-ylo, 1e-6);
    ylo = ylo - 0.08*yrng;
    yhi = yhi + 0.10*yrng;
end
ylim(ax, [ylo, yhi]);

text(ax, 0.05, 0.93, sprintf('\\textbf{bus %d}', op(kDisplay).bus_id), ...
    'Units','normalized', 'HorizontalAlignment','left', 'VerticalAlignment','top', ...
    'Color', C.dark);
text(ax, 0.05, 0.80, sprintf('$g_{\\min}=%.3g$, $g_0=%.3g$', ...
    gmin(kDisplay), g0_LF(kDisplay)), ...
    'Units','normalized', 'HorizontalAlignment','left', 'VerticalAlignment','top', ...
    'Color', C.midgray);
text(ax, f_eta_min, eta_no_g_min, sprintf('  $f^*=%.3g$ Hz', f_eta_min), ...
    'Color', C.midgray, 'HorizontalAlignment','left', 'VerticalAlignment','bottom');

% Inset: magnified view around the worst shortfall frequency. If the worst
% point is near DC, the inset shows the low-frequency deficit; otherwise it
% shows a local finite-frequency zoom.
drawnow;
axpos = ax.Position;
inset = axes('Position', [axpos(1)+0.52*axpos(3), axpos(2)+0.15*axpos(4), ...
                          0.34*axpos(3), 0.34*axpos(4)], ...
             'Color', [1 1 1]);
box(inset,'on'); hold(inset,'on');
span = cfg.fig3b_inset_peak_span;
f_in_lo = max(min(fHz), f_eta_min/span);
f_in_hi = min(max(fHz), max(f_eta_min*span, 3*f_eta_min));
if f_in_hi <= f_in_lo || ~isfinite(f_in_hi)
    f_in_lo = min(fHz); f_in_hi = min(max(fHz), 1e-2);
end
idx_in = (fHz >= f_in_lo) & (fHz <= f_in_hi) & idx_plot_b;
if nnz(idx_in) >= 5
    semilogx(inset, fHz(idx_in), eta_no_g(idx_in), '-',  'Color', C.dark, 'LineWidth',1.00);
    semilogx(inset, fHz(idx_in), eta_with_g(idx_in), '--', 'Color', C.blue, 'LineWidth',1.00);
    yline(inset, 0, '-', 'Color', C.gray, 'LineWidth',0.70);
    yline(inset, -g0_LF(kDisplay), ':', 'Color', C.midgray, 'LineWidth',0.80);
    xline(inset, f_eta_min, ':', 'Color', C.midgray, 'LineWidth',0.80, 'HandleVisibility','off');
    inset.XColor = C.dark; inset.YColor = C.dark;
    inset.FontSize = 7.0; inset.TickDir = 'out'; inset.LineWidth = 0.70;
    inset.Layer = 'top';
    grid(inset,'on');
    inset.GridColor = C.grid;
    inset.GridAlpha = 0.45;
    inset.XMinorGrid = 'off';
    inset.YMinorGrid = 'off';
    xlim(inset, [f_in_lo f_in_hi]);
    vals = [eta_no_g(idx_in); eta_with_g(idx_in); -g0_LF(kDisplay); 0];
    vals = vals(isfinite(vals));
    if ~isempty(vals)
        ylo2 = min(vals); yhi2 = max(vals);
        ypad2 = 0.12 * max(yhi2 - ylo2, 1e-6);
        ylim(inset, [ylo2 - ypad2, yhi2 + ypad2]);
    end
    xt = logspace(log10(f_in_lo), log10(f_in_hi), 3);
    xticks(inset, xt);
    xticklabels(inset, arrayfun(@(x) sprintf('%.1g', x), xt, 'UniformOutput', false));
end

% (c) Rightmost eigenvalues, spanning the entire lower row
ax = nexttile(tl,3,[1 2]);
[he0, he1] = plot_rightmost_eigs(ax, eig0, eig1, 60, C);
apply_axes_trans(ax);
xlabel(ax, 'Real part (1/s)');
ylabel(ax, 'Imaginary part (rad/s)');
add_panel_label(ax, '(c)');
leg = legend(ax, [he0 he1], {'Without virtual shunt','With virtual shunt'}, 'Location','northeast');
style_legend(leg);
title(ax, 'Rightmost closed-loop eigenvalues', 'FontWeight','normal', 'Interpreter','latex');

drawnow;
if isfield(cfg, 'export') && isfield(cfg.export, 'enable') && cfg.export.enable
    save_figure_multi(fig, cfg.export.basename, cfg.export.resolution);
    fprintf('=== Figure prepared and saved as %s.[png|pdf|fig] ===\n', cfg.export.basename);
else
    fprintf('=== Figure prepared. No image file was saved automatically. ===\n');
end

%% =========================================================================
%% Local functions

function set_plot_defaults_trans_style()
C = trans_colors();
set(groot,'defaultFigureColor','w');
set(groot,'defaultAxesColor','w');
set(groot,'defaultAxesColorOrder', [C.dark; C.blue; C.midgray; C.gray]);
set(groot,'defaultLineLineWidth',1.15);
set(groot,'defaultAxesFontSize',9);
set(groot,'defaultTextFontSize',9);
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextFontName','Times New Roman');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultAxesLineWidth',0.80);
set(groot,'defaultAxesTickDir','out');
set(groot,'defaultAxesTickLength',[0.012 0.012]);
end

function apply_axes_trans(ax)
C = trans_colors();
grid(ax,'on');
ax.GridLineStyle = '-';
ax.GridColor = C.grid;
ax.GridAlpha = 0.45;
ax.XMinorGrid = 'off';
ax.YMinorGrid = 'off';
ax.Box = 'on';
ax.Layer = 'top';
ax.XColor = C.dark;
ax.YColor = C.dark;
ax.LineWidth = 0.80;
end

function add_panel_label(ax, txt)
C = trans_colors();
text(ax, 0.02, 0.96, sprintf('\\textbf{%s}', txt), ...
    'Units','normalized', 'HorizontalAlignment','left', 'VerticalAlignment','top', ...
    'Color', C.dark);
end

function style_legend(leg)
if isempty(leg) || ~isgraphics(leg), return; end
leg.Box = 'off';
leg.Color = 'none';
leg.FontSize = 8.0;
end


function C = trans_colors()
C.blue       = [0.000 0.325 0.615];
C.red        = [0.78 0.10 0.10];
C.dark       = [0.12 0.12 0.12];
C.midgray    = [0.42 0.42 0.42];
C.gray       = [0.62 0.62 0.62];
C.lightgray  = [0.82 0.82 0.82];
C.grid       = [0.90 0.90 0.90];
C.barfill    = [0.92 0.92 0.92];
C.barfill_hi = [0.84 0.90 0.96];
C.baredge    = [0.62 0.62 0.62];
C.shade      = [0.97 0.97 0.97];
end

function save_figure_multi(fig, base_name, dpi_value)
if nargin < 3 || isempty(dpi_value)
    dpi_value = 600;
end
savefig(fig, [base_name '.fig']);
try
    exportgraphics(fig, [base_name '.png'], 'Resolution', dpi_value);
    exportgraphics(fig, [base_name '.pdf'], 'ContentType', 'vector');
catch
    print(fig, [base_name '.png'], '-dpng', sprintf('-r%d', dpi_value));
    print(fig, [base_name '.pdf'], '-dpdf', '-painters');
end
end

function fig = create_fig_inch(name, w_in, h_in)
fig = figure('Name',name,'Renderer','painters','Units','inches','Position',[1 1 w_in h_in]);
set(fig,'Color','w', 'InvertHardcopy','off');
end
function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end

function print_network_eigen_summary(eig_net, w0)
% Print the eigenvalue signature expected for the no-ground-admittance network.
tolReal = 1e-7;
tolFreq = 1e-5*max(1,w0);
re = real(eig_net); im = imag(eig_net);
idx_cm = find(abs(re) < tolReal & abs(abs(im)-w0) < tolFreq);
fprintf('  Isolated network subsystem: maxRe(eigs)=%.4e, #(|Re|<%.1e)= %d\n', ...
    max(re), tolReal, sum(abs(re)<tolReal));
if numel(idx_cm) >= 2
    fprintf('  Common-mode imaginary-axis pair detected near s = +/- j*w0:\n');
    [~,ord] = sort(im(idx_cm));
    idx_cm = idx_cm(ord);
    for kk = 1:min(numel(idx_cm),4)
        lam = eig_net(idx_cm(kk));
        fprintf('      %+.6e %+.6ej  rad/s\n', real(lam), imag(lam));
    end
else
    warning('Expected common-mode pair near +/-j*w0 was not clearly detected. Check Y_L, GS/BS, and incidence connectivity.');
end

% Print a few rightmost modes excluding exact common-mode pair only for context.
[~,idx] = maxk(re, min(8,numel(eig_net)));
fprintf('  Rightmost isolated-network eigenvalues:\n');
for kk = 1:numel(idx)
    lam = eig_net(idx(kk));
    fprintf('      %+.6e %+.6ej\n', real(lam), imag(lam));
end
end

function print_closed_loop_eigen_summary(name, eigvals, K)
if nargin < 3, K = 10; end
re = real(eigvals);
numRHP = sum(re > 1e-8);
numMarg = sum(abs(re) <= 1e-8);
fprintf('  %s: maxRe=%.4e, #RHP=%d, #near-imag-axis=%d\n', ...
    name, max(re), numRHP, numMarg);
[~,idx] = maxk(re, min(K,numel(eigvals)));
fprintf('  Rightmost eigenvalues of %s:\n', name);
for kk = 1:numel(idx)
    lam = eigvals(idx(kk));
    fprintf('      %+.6e %+.6ej\n', real(lam), imag(lam));
end
end


function write_numeric_summary(filename, cfg, eig_net, eig0, eig1, op, gmin, g0_LF, g_vec, kDisplay)
% Write a compact plain-text report with the numbers needed to answer the supervisor.
try
    fid = fopen(filename, 'w');
    if fid < 0, warning('Could not open summary file %s for writing.', filename); return; end
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, 'No-ground-admittance IEEE 39-bus case-study summary\n');
    fprintf(fid, '===================================================\n');
    fprintf(fid, 'f0 = %.6f Hz, w0 = %.12f rad/s\n', cfg.f0, cfg.w0);
    fprintf(fid, 'Network model: no static Y_L, no bus GS/BS shunts, no grounding floor.\n\n');
    [~, idx_plus]  = min(abs(eig_net - 1j*cfg.w0));
    [~, idx_minus] = min(abs(eig_net + 1j*cfg.w0));
    lam_plus  = eig_net(idx_plus);
    lam_minus = eig_net(idx_minus);
    idx_common = false(size(eig_net)); idx_common(idx_plus)=true; idx_common(idx_minus)=true;
    fprintf(fid, '1) Isolated network subsystem\n');
    fprintf(fid, 'Expected common-mode pair: lambda = +/- j*w0 = +/- j %.12f rad/s\n', cfg.w0);
    fprintf(fid, 'Computed eigenvalue near +j*w0: %+.12e %+.12ej\n', real(lam_plus), imag(lam_plus));
    fprintf(fid, 'Computed eigenvalue near -j*w0: %+.12e %+.12ej\n', real(lam_minus), imag(lam_minus));
    fprintf(fid, 'Max real part, all network modes: %.12e\n', max(real(eig_net)));
    if any(~idx_common), fprintf(fid, 'Max real part, excluding this pair: %.12e\n\n', max(real(eig_net(~idx_common)))); end
    fprintf(fid, '2) Closed loop without virtual shunt\n');
    fprintf(fid, 'max real part = %.12e\n', max(real(eig0)));
    fprintf(fid, '#RHP eigenvalues, tol=1e-8 = %d\n', sum(real(eig0)>1e-8));
    [~, idx0] = maxk(real(eig0), min(12,numel(eig0)));
    for k = 1:numel(idx0), lam=eig0(idx0(k)); fprintf(fid, '  %+.12e %+.12ej\n', real(lam), imag(lam)); end
    fprintf(fid, '\n3) Closed loop with virtual shunt\n');
    fprintf(fid, 'max real part = %.12e\n', max(real(eig1)));
    fprintf(fid, '#RHP eigenvalues, tol=1e-8 = %d\n', sum(real(eig1)>1e-8));
    [~, idx1] = maxk(real(eig1), min(12,numel(eig1)));
    for k = 1:numel(idx1), lam=eig1(idx1(k)); fprintf(fid, '  %+.12e %+.12ej\n', real(lam), imag(lam)); end
    fprintf(fid, '\n4) Display converter and local passivation gains\n');
    fprintf(fid, 'Display bus = %d\n', op(kDisplay).bus_id);
    fprintf(fid, 'gmin(display) = %.12e\n', gmin(kDisplay));
    fprintf(fid, 'g0_LF(display) = %.12e\n', g0_LF(kDisplay));
    fprintf(fid, 'g_applied(display) = %.12e\n\n', g_vec(kDisplay));
    fprintf(fid, 'All bus gains: bus, P0, Q0, g0_LF, gmin, g_applied\n');
    for k = 1:numel(op)
        fprintf(fid, '%3d  %+.8e  %+.8e  %.8e  %.8e  %.8e\n', op(k).bus_id, op(k).P0, op(k).Q0, g0_LF(k), gmin(k), g_vec(k));
    end
    fprintf('  Numeric summary written to %s\n', filename);
catch ME
    warning('Failed to write numeric summary: %s', ME.message);
end
end

function [res, stress_used] = run_pf_stressed_safe(case_name, stress, mpopt, paper_match)
define_constants;
mpc0 = loadcase(case_name);

% Apply branch stress
mpc0.branch(:, BR_R) = mpc0.branch(:, BR_R) * stress.r_scale;
mpc0.branch(:, BR_X) = mpc0.branch(:, BR_X) * stress.x_scale;
% Keep BR_B unchanged (line charging not scaled)

% Paper-matching network preprocessing
if nargin >= 4 && isstruct(paper_match)
    if isfield(paper_match, 'remove_taps_and_shifts') && paper_match.remove_taps_and_shifts
        mpc0.branch(:, SHIFT) = 0;
        ix_tap = find(mpc0.branch(:, TAP) ~= 0);
        mpc0.branch(ix_tap, TAP) = 1;
    end
    if isfield(paper_match, 'remove_bus_shunts') && paper_match.remove_bus_shunts
        % Remove static bus shunts from the PF model so that the operating point
        % is consistent with the no-ground-admittance dynamic network.
        % Branch charging BR_B is not removed.
        mpc0.bus(:, GS) = 0;
        mpc0.bus(:, BS) = 0;
    end
    if isfield(paper_match, 'gs_floor_pu') && paper_match.gs_floor_pu > 0
        gs_floor_MW = paper_match.gs_floor_pu * mpc0.baseMVA;
        mpc0.bus(:, GS) = max(mpc0.bus(:, GS), gs_floor_MW);
    end
else
    paper_match.remove_taps_and_shifts = false;
    paper_match.remove_bus_shunts = false;
    paper_match.gs_floor_pu = 0;
end

% Try PF with load scaling backoff if needed
load_scale = stress.load_scale;
for k = 1:12
    mpc = mpc0;

    % Scale loads
    mpc.bus(:, PD) = mpc0.bus(:, PD) * load_scale;
    mpc.bus(:, QD) = mpc0.bus(:, QD) * load_scale;

    % Scale Pg setpoints roughly with load (helps convergence / keeps flows similar)
    mpc.gen(:, PG) = mpc0.gen(:, PG) * load_scale;

    res = runpf(mpc, mpopt);
    if res.success
        stress_used = stress;
        stress_used.load_scale_used = load_scale;
        stress_used.remove_taps_and_shifts = paper_match.remove_taps_and_shifts;
        if isfield(paper_match,'remove_bus_shunts')
            stress_used.remove_bus_shunts = paper_match.remove_bus_shunts;
        else
            stress_used.remove_bus_shunts = false;
        end
        stress_used.gs_floor_pu = paper_match.gs_floor_pu;
        return;
    end
    load_scale = 0.9 * load_scale;
end
error('Power flow failed to converge even after load backoff.');
end

function net = build_network_ss_dq(res, w0, opts)
define_constants;
if ~isfield(opts, 'assert_positive_G'), opts.assert_positive_G = false; end
if ~isfield(opts, 'use_static_load_admittance'), opts.use_static_load_admittance = false; end

bus = res.bus;
branch_all = res.branch;
baseMVA = res.baseMVA;

% Keep only in-service branches
branch = branch_all(branch_all(:, BR_STATUS) == 1, :);

bus_ids = bus(:, BUS_I);
N = size(bus,1);
M = size(branch,1);

J = sparse([0 1; -1 0]);
I2 = speye(2);

% Map branch endpoints to internal indices
[~, f_idx] = ismember(branch(:, F_BUS), bus_ids);
[~, t_idx] = ismember(branch(:, T_BUS), bus_ids);
assert(all(f_idx>0 & t_idx>0), 'Branch endpoint mapping failed.');

% --- Bus shunt capacitance from line charging + (capacitive part of) bus BS ---
Bcap = zeros(N,1); % pu susceptance (capacitive part only for dynamic C)
for k = 1:M
    b = branch(k, BR_B);              % total line charging susceptance (pu)
    if b == 0, continue; end
    tau = branch(k, TAP); if tau == 0, tau = 1; end
    Bcap(f_idx(k)) = Bcap(f_idx(k)) + 0.5*b/(tau^2);
    Bcap(t_idx(k)) = Bcap(t_idx(k)) + 0.5*b;
end
Bbus = bus(:, BS)/baseMVA;            % pu
Bcap = Bcap + max(0, Bbus);           % only capacitive part to C

c = Bcap / w0;                        % C = B/omega  (pu*s)
c(c < opts.c_min) = opts.c_min;

% --- Optional static load/ground admittance Y_L ---
% For the no-ground-admittance test required by the proof, Y_L must be zero.
% Constant-power loads from the PF are instead absorbed into the converter
% operating points. Set opts.use_static_load_admittance=true only for the
% loaded-network variant.
if opts.use_static_load_admittance
    Vm = bus(:, VM);
    V2 = max(Vm.^2, 1e-8);

    Ppu = bus(:, PD)/baseMVA;
    Qpu = bus(:, QD)/baseMVA;

    G = Ppu ./ V2;                    % (P - jQ)/|V|^2 => G = P/|V|^2
    B = -Qpu ./ V2;                   % susceptance part (note sign)

    % Add bus shunt conductance GS, and static part of bus BS.
    G = G + bus(:, GS)/baseMVA;
    B = B + min(0, Bbus);             % inductive part kept static if enabled
    G = G + opts.g_eps;
else
    G = zeros(N,1);
    B = zeros(N,1);
end

if opts.assert_positive_G && any(G <= 0)
    warning('Some buses have nonpositive static conductance G.');
end

% YL block in dq: i = (G*I - B*J) v
YL = spalloc(2*N, 2*N, 4*N);
for i = 1:N
    blk = G(i)*I2 - B(i)*J;
    YL(2*i-1:2*i, 2*i-1:2*i) = blk;
end

% --- Tap/phase-shift aware incidence matrices ---
% v_ell,k = (1/tau)*R(-phi)*v_f - v_t
% i_inj_f += (1/tau)*R(+phi)*i_k,    i_inj_t += -i_k
E_v = spalloc(2*M, 2*N, 4*2*M);
E_i = spalloc(2*N, 2*M, 4*2*M);

for k = 1:M
    tau = branch(k, TAP); if tau == 0, tau = 1; end
    phi = branch(k, SHIFT)*pi/180;    % rad

    Rm = [cos(phi)  sin(phi);
          -sin(phi) cos(phi)];        % R(-phi)
    A = (1/tau) * Rm;
    Bm = A.';                         % (1/tau)*R(+phi)

    rf = 2*f_idx(k)-1 : 2*f_idx(k);
    rt = 2*t_idx(k)-1 : 2*t_idx(k);
    rk = 2*k-1 : 2*k;

    E_v(rk, rf) = A;
    E_v(rk, rt) = -eye(2);

    E_i(rf, rk) = Bm;
    E_i(rt, rk) = -eye(2);
end

% --- Branch R and L ---
r = branch(:, BR_R);
x = branch(:, BR_X);
L = x / w0;                           % L = X/omega (pu*s)
L(L < 1e-10) = 1e-10;

Cinv = kron(spdiags(1./c, 0, N, N), I2);
Linv = kron(spdiags(1./L, 0, M, M), I2);
RovL = kron(spdiags(r./L, 0, M, M), I2);

JN = kron(speye(N), J);
JM = kron(speye(M), J);

A_vv = w0*JN - Cinv*YL;
A_vi = -Cinv*E_i;
B_v  = Cinv;

A_iv = Linv*E_v;
A_ii = w0*JM - RovL;

net.N = N; net.M = M;
net.A_vv = A_vv; net.A_vi = A_vi; net.B_v = B_v;
net.A_iv = A_iv; net.A_ii = A_ii;

% Store for Y_net_tot(jw) evaluation
net.c = c;
net.YL = YL;
net.Ev = E_v;
net.Ei = E_i;
net.r = r;
net.L = L;
end

function [lam_curve, lam_min, f_at_min] = network_port_passivity_curve(net, w0, w_grid, port_bus_idx)
% Compute lambda_min(sym(Y_net,port(jw))) over w_grid (rad/s),
% where Y_net,port is Kron-reduced to the converter-bus port set.
N = net.N;
I2 = eye(2);
J  = [0 1; -1 0];

% Build index sets (dq dof indices)
port_dof = [];
for k = 1:numel(port_bus_idx)
    i = port_bus_idx(k);
    port_dof = [port_dof, 2*i-1, 2*i]; %#ok<AGROW>
end
all_dof = 1:(2*N);
int_dof = setdiff(all_dof, port_dof);

% Constant C blocks
Cblk  = kron(diag(net.c), I2);
CJblk = kron(diag(net.c), J);

% Branch blocks
Lblk = diag(net.L);
Rblk = diag(net.r);

lam_curve = zeros(numel(w_grid),1);

for ii = 1:numel(w_grid)
    w = w_grid(ii);
    s = 1j*w;

    % Z(s) = (sL + R)⊗I - w0 L⊗J
    Z = kron(s*Lblk + Rblk, I2) - w0*kron(Lblk, J);

    % Y_lines = Ei * Z^{-1} * Ev
    Y_lines = net.Ei * (Z \ net.Ev);

    % Full nodal admittance
    Y_tot = s*Cblk - w0*CJblk + Y_lines + net.YL;

    % Kron reduction to port buses: Yport = YPP - YPQ*YQQ^{-1}*YQP
    YPP = Y_tot(port_dof, port_dof);
    if isempty(int_dof)
        Yport = YPP;
    else
        YPQ = Y_tot(port_dof, int_dof);
        YQP = Y_tot(int_dof, port_dof);
        YQQ = Y_tot(int_dof, int_dof);
        Yport = YPP - YPQ * (YQQ \ YQP);
    end

    S = 0.5*(Yport + Yport');
    lam_curve(ii) = min(real(eig(full(S))));
end

[lam_min, idx] = min(lam_curve);
f_at_min = w_grid(idx)/(2*pi);
end

function [conv_bus_ids, conv_bus_idx, S] = select_converter_buses(res, use_gen_only)
define_constants;
bus_ids = res.bus(:, BUS_I);
if use_gen_only
    conv_bus_ids = unique(res.gen(:, GEN_BUS));
else
    conv_bus_ids = bus_ids;
end
[tf, conv_bus_idx] = ismember(conv_bus_ids, bus_ids);
assert(all(tf), 'Converter bus mapping failed.');
Nc = numel(conv_bus_idx);

N = numel(bus_ids);
S = sparse(2*N, 2*Nc);
for k = 1:Nc
    i = conv_bus_idx(k);
    S(2*i-1, 2*k-1) = 1;
    S(2*i  , 2*k  ) = 1;
end
end

function op = build_converter_operating_points(res, conv_bus_idx, mode)
% Build converter operating points.
%
% mode = 'gen_buses_inject_only'
%   - Generator buses inject the solved generator powers from MATPOWER.
%   - Non-generator buses have zero nominal converter injection.
%   - This mode is used only when static load admittances Y_L are retained
%     in the dynamic network.
%
% mode = 'net_bus_injection_no_ground'
%   - Static load admittances are removed from the dynamic network.
%   - Constant-power loads from the PF are absorbed into local converter
%     operating-point injections: S_conv = S_gen - S_load.

define_constants;
baseMVA = res.baseMVA;
bus_ids = res.bus(:, BUS_I);
N = size(res.bus,1);

Vm = res.bus(:, VM);
Va = res.bus(:, VA) * pi/180;
V  = Vm .* exp(1j*Va);

% Aggregate generator injections per bus (pu on system base)
gen_bus_idx = zeros(size(res.gen,1),1);
for gg = 1:size(res.gen,1)
    [~, bi] = ismember(res.gen(gg, GEN_BUS), bus_ids);
    gen_bus_idx(gg) = bi;
end
Pg_bus = accumarray(gen_bus_idx, res.gen(:, PG)/baseMVA, [N 1], @sum, 0);
Qg_bus = accumarray(gen_bus_idx, res.gen(:, QG)/baseMVA, [N 1], @sum, 0);

switch lower(mode)
    case 'gen_buses_inject_only'
        % Used only when the dynamic network explicitly retains the static load model Y_L.
        % Non-generator buses have zero nominal converter injection.
        Sconv_bus = Pg_bus + 1j*Qg_bus;

    case 'net_bus_injection_no_ground'
        % Used with the no-ground-admittance dynamic network.
        % Since static load admittances are removed from the network model, constant-power
        % loads from the PF are represented as negative local converter injections.
        % Thus each all-bus converter carries the net complex bus injection.
        Pload_bus = res.bus(:, PD)/baseMVA;
        Qload_bus = res.bus(:, QD)/baseMVA;
        Sconv_bus = (Pg_bus - Pload_bus) + 1j*(Qg_bus - Qload_bus);

    otherwise
        error('Unknown operating-point mode: %s', mode);
end

gen_bus_id_set = unique(res.gen(:, GEN_BUS));

Nc = numel(conv_bus_idx);
op = repmat(struct('v0',[],'i0',[],'delta0',0,'E0',0,'P0',0,'Q0',0, ...
                   'bus_idx',0,'bus_id',0,'is_gen_bus',false), Nc, 1);

for k = 1:Nc
    bi = conv_bus_idx(k);

    Vk = V(bi);
    Sk = Sconv_bus(bi);                % injection assigned to converter (pu)
    Ik = conj(Sk / Vk);                % I = conj(S/V)

    op(k).v0 = [real(Vk); imag(Vk)];
    op(k).i0 = [real(Ik); imag(Ik)];
    op(k).delta0 = angle(Vk);
    op(k).E0 = abs(Vk);
    op(k).P0 = real(Sk);
    op(k).Q0 = imag(Sk);
    op(k).bus_idx = bi;
    op(k).bus_id  = bus_ids(bi);
    op(k).is_gen_bus = ismember(bus_ids(bi), gen_bus_id_set);
end
end

function prm_vec = make_buswise_params(op, prm_base, cfg)
% Bus-wise parameter profile used in the all-bus case.
% Generator buses keep the paper's GFM droop/filter parameters.
% Non-generator buses are modeled with milder voltage-support GFM parameters,
% which avoids unrealistically large passivation gains caused by applying
% the same aggressive generator-style outer droop at every bus. In the
% no-ground setting their operating-point injections may be negative net loads.
Nc = numel(op);
prm_vec = repmat(prm_base, Nc, 1);
if isfield(cfg, 'support_profile') && isfield(cfg.support_profile, 'enable') && cfg.support_profile.enable
    for k = 1:Nc
        if ~op(k).is_gen_bus
            prm_vec(k).dp    = cfg.support_profile.dp_non_gen;
            prm_vec(k).dq    = cfg.support_profile.dq_non_gen;
            prm_vec(k).tau_p = cfg.support_profile.tau_p_non_gen;
            prm_vec(k).tau_q = cfg.support_profile.tau_q_non_gen;
        end
    end
end
end

function [prm_out, conv, worstReA] = tune_voltage_loop_all(op, prm_in, w0, tune, cfg)
% Search for a locally stable voltage-loop tuning for all converter models.
% Strategy:
%   1) try the user-specified grid first;
%   2) if that fails, automatically expand the Kpv/Kiv search grid;
%   3) if still needed, mildly relax dp as a last resort.
%
% The selected tuning is the FIRST one (in ascending-gain order) that makes
% every local converter internally stable with the requested margin. This
% avoids unnecessarily aggressive gains.

if ~isfield(tune, 'auto_expand'),   tune.auto_expand = true;   end
if ~isfield(tune, 'allow_dp_relax'), tune.allow_dp_relax = true; end

userKpv = sort(unique(tune.Kpv_list(:).'));
userKiv = sort(unique(tune.Kiv_list(:).'));

search_sets = {};
search_sets{end+1} = struct( ...
    'name', 'user grid', ...
    'Kpv_list', userKpv, ...
    'Kiv_list', userKiv, ...
    'dp_scale', 1.0);

if tune.auto_expand
    extKpv = sort(unique([userKpv, 0.8 1.0 1.2 1.5 2 3 5 8 10 15 20 30]));
    extKiv = sort(unique([userKiv, 1e-3 2e-3 5e-3 0.01 0.02 0.05 0.1 0.2 0.5 1 2 5 10 20 30]));

    search_sets{end+1} = struct( ...
        'name', 'expanded grid', ...
        'Kpv_list', extKpv, ...
        'Kiv_list', extKiv, ...
        'dp_scale', 1.0);

    if tune.allow_dp_relax
        for sc = [0.7 0.5 0.3]
            search_sets{end+1} = struct( ...
                'name', sprintf('expanded grid with dp scale %.2f', sc), ...
                'Kpv_list', extKpv, ...
                'Kiv_list', extKiv, ...
                'dp_scale', sc);
        end
    end
end

bestWorst = Inf;
bestInfo  = struct('Kpv', NaN, 'Kiv', NaN, 'dp', prm_in.dp, 'setname', '');

for ss = 1:numel(search_sets)
    sset = search_sets{ss};
    prm_base = prm_in;
    prm_base.dp = prm_in.dp * sset.dp_scale;

    for Kpv = sset.Kpv_list
        for Kiv = sset.Kiv_list
            prm = prm_base;
            prm.Kpv = Kpv;
            prm.Kiv = Kiv;

            conv_try = build_all_converters(op, prm, w0, cfg);
            reA = [conv_try.maxReA];
            worst = max(reA);

            if worst < bestWorst
                bestWorst = worst;
                bestInfo.Kpv = Kpv;
                bestInfo.Kiv = Kiv;
                bestInfo.dp  = prm.dp;
                bestInfo.setname = sset.name;
            end

            if worst <= -tune.stab_margin
                prm_out = prm;
                conv = conv_try;
                worstReA = worst;

                if ss > 1
                    warning(['Voltage-loop tuning grid was auto-expanded to recover local stability. ' ...
                             'Selected Kpv=%.4g, Kiv=%.4g, dp=%.4g from %s.'], ...
                             prm_out.Kpv, prm_out.Kiv, prm_out.dp, sset.name);
                end
                if sset.dp_scale < 1.0
                    warning(['Droop gain dp was reduced from %.4g to %.4g as a last-resort ' ...
                             'local-stability fix for the all-bus converter case.'], ...
                             prm_in.dp, prm_out.dp);
                end
                return;
            end
        end
    end
end

error(['Cannot make all converters internally stable. Best search result gives ' ...
       'maxRe(A_conv)=%.3e at Kpv=%.4g, Kiv=%.4g, dp=%.4g (%s).'], ...
       bestWorst, bestInfo.Kpv, bestInfo.Kiv, bestInfo.dp, bestInfo.setname);
end

function conv = build_all_converters(op, prm, w0, cfg)
Nc = numel(op);
conv = repmat(struct('A',[],'B',[],'C',[],'D',[],'sysY_dev',[],'par',[],'maxReA',0), Nc, 1);

prm_vec = make_buswise_params(op, prm, cfg);
for k = 1:Nc
    [A,B,C,D,maxReA] = build_gfm_ss(op(k), prm_vec(k), w0);
    conv(k).A = A; conv(k).B = B; conv(k).C = C; conv(k).D = D;
    conv(k).sysY_dev = ss(A,B,C,D);
    conv(k).par = prm_vec(k);
    conv(k).maxReA = maxReA;
end
end

function [A,B,C,D,maxReA] = build_gfm_ss(op, prm, w0)
% States: x = [ i(2); xiI(2); xiV(2); delta(1); xp(1); xq(1) ] => 9 states
J = [0 1; -1 0];
I2 = eye(2);

Lf = prm.Xf / w0;
rf = prm.rf;

v0 = op.v0(:);
i0 = op.i0(:);
delta0 = op.delta0;
E0 = op.E0;

ucc0 = v0 + rf*i0 - (w0*Lf)*J*i0;
xiV0 = i0;
xiI0 = ucc0;

P_E = [cos(delta0); sin(delta0)];
P_d = [-E0*sin(delta0); E0*cos(delta0)];

idx_i   = 1:2;
idx_xiI = 3:4;
idx_xiV = 5:6;
idx_del = 7;
idx_xp  = 8;
idx_xq  = 9;

A = zeros(9,9);
B = zeros(9,2);

% i_dot: Lf (i_dot - w0 J i) = u - v - rf i
A(idx_i, idx_i)   = w0*J - ((rf + prm.Kpc)/Lf)*I2;
A(idx_i, idx_xiV) = (prm.Kpc/Lf)*I2;
A(idx_i, idx_xiI) = (1/Lf)*I2;
A(idx_i, idx_del) = (prm.Kpc*prm.Kpv/Lf) * P_d;
A(idx_i, idx_xq)  = (-prm.Kpc*prm.Kpv*prm.dq/Lf) * P_E;
B(idx_i, :)       = -((prm.Kpc*prm.Kpv + 1)/Lf)*I2;

% xiI_dot: xiI_dot = -delta_dot J xiI0 + Kic (i* - i)
A(idx_xiI, idx_i)   = -prm.Kic*I2;
A(idx_xiI, idx_xiV) =  prm.Kic*I2;
A(idx_xiI, idx_del) = (prm.Kic*prm.Kpv) * P_d;
A(idx_xiI, idx_xq)  = (-prm.Kic*prm.Kpv*prm.dq) * P_E;
A(idx_xiI, idx_xp)  = (prm.dp) * (J*xiI0);
B(idx_xiI, :)       = -(prm.Kic*prm.Kpv)*I2;

% xiV_dot: xiV_dot = -delta_dot J xiV0 + Kiv (v* - v)
A(idx_xiV, idx_del) = (prm.Kiv) * P_d;
A(idx_xiV, idx_xq)  = (-prm.Kiv*prm.dq) * P_E;
A(idx_xiV, idx_xp)  = (prm.dp) * (J*xiV0);
B(idx_xiV, :)       = -(prm.Kiv)*I2;

% delta_dot = -dp * xp
A(idx_del, idx_xp) = -prm.dp;

% xp_dot = (p - xp)/tau_p,  p = v0^T i + i0^T v
A(idx_xp, idx_i)  = (1/prm.tau_p) * (v0.');
A(idx_xp, idx_xp) = -1/prm.tau_p;
B(idx_xp, :)      = (1/prm.tau_p) * (i0.');

% xq_dot = (q - xq)/tau_q,  q = -v0^T J i + i0^T J v
A(idx_xq, idx_i)  = (1/prm.tau_q) * (-v0.'*J);
A(idx_xq, idx_xq) = -1/prm.tau_q;
B(idx_xq, :)      = (1/prm.tau_q) * (i0.'*J);

% Output: i_dev = i
C = [I2 zeros(2,7)];
D = zeros(2,2);

evA = eig(A);
maxReA = max(real(evA));
end

function Acl = assemble_global_A(net, conv, S, g_vec)
% Global state: x = [v_bus(2N); i_line(2M); x_conv(9Nc)]
N = net.N; M = net.M; Nc = numel(conv);
nv = 2*N; ni = 2*M; nx = 9*Nc; %#ok<NASGU>

A_c = sparse(nx, nx);
B_c = sparse(nx, 2*Nc);
C_c = sparse(2*Nc, nx);

for k = 1:Nc
    ix = (9*(k-1)+1):(9*k);
    iu = (2*(k-1)+1):(2*k);

    A_c(ix,ix) = conv(k).A;
    B_c(ix,iu) = conv(k).B;
    C_c(iu,ix) = conv(k).C;
end

% Virtual shunt in SMALL SIGNAL: Δi_inj = i_conv - g Δv  (injection into network)
D_c = kron(spdiags(-g_vec, 0, Nc, Nc), speye(2));

A_vv_eff = net.A_vv + (net.B_v * S) * D_c * (S.');
A_vx     = (net.B_v * S) * C_c;
A_xv     = B_c * (S.');

Acl = [ A_vv_eff, net.A_vi, A_vx;
        net.A_iv, net.A_ii, sparse(2*M, 9*Nc);
        A_xv,     sparse(9*Nc, 2*M), A_c ];
end

function [gmin, fpeak_Hz] = compute_gmin_from_ss_refined(sysY, w_grid, g0_LF)
% Two-pass frequency scan: a coarse global scan followed by local
% refinement around the dominant peak of lambda_max(sym(Y_dev(jw))).
[lambdaMax, ~] = lambda_max_curve(sysY, w_grid);
[lam_peak, idx_peak] = max(lambdaMax);
w_peak = w_grid(idx_peak);

w_lo = max(w_grid(1),  w_peak/10);
w_hi = min(w_grid(end), w_peak*10);
if w_hi > w_lo
    w_ref = unique([w_grid(:); logspace(log10(w_lo), log10(w_hi), 1200).']);
    [lambdaMax_ref, ~] = lambda_max_curve(sysY, w_ref);
    [lam_peak, idx_peak] = max(lambdaMax_ref);
    w_peak = w_ref(idx_peak);
else
    w_ref = w_grid; %#ok<NASGU>
end

gmin = max([0; g0_LF; lam_peak]);
fpeak_Hz = w_peak/(2*pi);
end

function [lambdaMax, fHz] = lambda_max_curve(sysY, w_grid)
Y = freqresp(sysY, w_grid);
lambdaMax = zeros(numel(w_grid),1);
for ii = 1:numel(w_grid)
    Yjw = Y(:,:,ii);
    S = 0.5*(Yjw + Yjw');
    lam = eig(S);
    lambdaMax(ii) = max(real(lam));
end
fHz = w_grid/(2*pi);
end

function [lambdaMax_w, fHz] = lambda_max_sym_vs_w(sysY, w_grid)
Y = freqresp(sysY, w_grid);
lambdaMax_w = zeros(numel(w_grid),1);
for ii = 1:numel(w_grid)
    Yjw = Y(:,:,ii);
    S = 0.5*(Yjw + Yjw');
    lam = eig(S);
    lambdaMax_w(ii) = max(real(lam));
end
fHz = w_grid/(2*pi);
end

function g0 = g0_from_DC_limit(op, dq)
% Theoretical LF asymptote: g0 = lambda_max( sym(Y_dev(0)) )
v0 = op.v0(:);
i0 = op.i0(:);
delta = atan2(v0(2), v0(1));
Q = [cos(-delta) -sin(-delta); sin(-delta) cos(-delta)];
v_al = Q*v0; i_al = Q*i0;

v = v_al(1);     % v_q ~ 0
id = i_al(1); iq = i_al(2);

g0 = sqrt( (id/v)^2 + (iq/v - 1/(2*dq*v))^2 );
end

function [sv_full, sv_mid, rel_err] = compare_mid_approx(prm, w0, sysY, w_grid)
J = [0 1; -1 0];
I2 = eye(2);

Lf = prm.Xf / w0;

Yfull = freqresp(sysY, w_grid);
sv_full = zeros(numel(w_grid),1);
sv_mid  = zeros(numel(w_grid),1);
rel_err = zeros(numel(w_grid),1);

for ii = 1:numel(w_grid)
    w = w_grid(ii);
    s = 1j*w;

    Yjw = Yfull(:,:,ii);
    sv_full(ii) = max(svd(Yjw));

    GI = prm.Kpc + prm.Kic/s;
    GV = prm.Kpv + prm.Kiv/s;
    mu = GI*GV;
    alpha = 1 + mu;

    ZL = (s*Lf + prm.rf)*I2 - (w0*Lf)*J;
    H  = (ZL + GI*I2) \ eye(2);

    % Mid-frequency approximation: Y_dev_mid ≈ -alpha * H
    Ymid = -(alpha) * H;
    sv_mid(ii) = max(svd(Ymid));

    rel_err(ii) = norm(Yjw - Ymid, 2) / max(norm(Yjw,2), 1e-12);
end
end

function t_end = choose_time_horizon(maxReUnstable, maxReStable)
% Choose a horizon that shows both the unstable growth and the stable decay
% without creating an unnecessarily extreme dynamic range in the semilog plot.
if nargin < 2, maxReStable = -1e-2; end

if maxReUnstable > 1e-6
    % target roughly 1e8 growth in the unstable trace
    t_growth = max(0.8, min(2.5, log(1e8)/maxReUnstable));
else
    t_growth = 2.0;
end

decay_rate = max(-maxReStable, 5e-3);
% show several stable time constants, but keep the window compact
 t_decay = max(1.5, min(4.0, 6/decay_rate));

t_end = max(t_growth, t_decay);
end

function [y0, y1, u, info] = simulate_voltage_response_current_disturbance(A0, A1, net, bus_idx, t, cfg)
% Zero-state response to a localized exogenous current-injection disturbance
% entering through the \Delta i_d channel in Fig. 2.
%
% Disturbance waveform:
%   u(t) = bar i_d sin(2*pi*f_d*t) w(t),   0 <= t <= T_b,
% and u(t)=0 for t>T_b, where w(t) is a smooth Hann envelope.
%
% This finite-duration sinusoidal current burst stays fully consistent with
% Fig. 2, provides a visually clear oscillatory excitation, and lets the
% uncompensated unstable mode emerge after the burst while the passivated
% response decays.

nTot = size(A0,1);
N = net.N;
extra = nTot - 2*N;

switch lower(cfg.time_response.axis)
    case 'd'
        busvec = zeros(2*N,1);
        busvec(2*bus_idx-1) = 1;
        Csel = sparse(1, nTot);
        Csel(1, 2*bus_idx-1) = 1;
    case 'q'
        busvec = zeros(2*N,1);
        busvec(2*bus_idx) = 1;
        Csel = sparse(1, nTot);
        Csel(1, 2*bus_idx) = 1;
    otherwise
        error('Unknown cfg.time_response.axis = %s', cfg.time_response.axis);
end

Bext = [net.B_v * busvec; zeros(extra,1)];

info.id_amp         = cfg.time_response.id_amp;
info.id_f_Hz        = cfg.time_response.id_f_Hz;
info.burst_duration = cfg.time_response.burst_duration;

tcol = t(:);
u = zeros(size(tcol));
Tb = info.burst_duration;
idx = (tcol <= Tb);
if any(idx)
    tau = tcol(idx);
    env = 0.5*(1 - cos(2*pi*tau/Tb));   % Hann envelope on [0,Tb]
    u(idx) = info.id_amp * sin(2*pi*info.id_f_Hz*tau) .* env;
end
if ~isempty(u), u(1) = 0; end

sys0 = ss(full(A0), full(Bext), full(Csel), 0);
sys1 = ss(full(A1), full(Bext), full(Csel), 0);

y0 = lsim(sys0, u, t, zeros(nTot,1));
y1 = lsim(sys1, u, t, zeros(nTot,1));

y0 = y0(:);
y1 = y1(:);
end


function [h0, h1] = plot_rightmost_eigs(ax, eig0, eig1, K, C)
re0 = real(eig0); im0 = imag(eig0);
re1 = real(eig1); im1 = imag(eig1);

K0 = min(K, numel(eig0)); K1 = min(K, numel(eig1));
[~, idx0] = maxk(re0, K0);
[~, idx1] = maxk(re1, K1);

idx0_stable = idx0(re0(idx0) <= 0);
idx0_rhp    = idx0(re0(idx0) > 0);

hold(ax,'on');
% Without-shunt eigenvalues are circles. Any sampled circle with positive
% real part is highlighted in red to make the unstable modes immediately visible.
if ~isempty(idx0_stable)
    h0 = plot(ax, re0(idx0_stable), im0(idx0_stable), 'o', ...
        'Color', C.dark, 'MarkerSize',5.2, 'LineWidth',0.95, 'MarkerFaceColor','w');
else
    h0 = plot(ax, NaN, NaN, 'o', ...
        'Color', C.dark, 'MarkerSize',5.2, 'LineWidth',0.95, 'MarkerFaceColor','w');
end
if ~isempty(idx0_rhp)
    plot(ax, re0(idx0_rhp), im0(idx0_rhp), 'o', ...
        'Color', C.red, 'MarkerSize',5.8, 'LineWidth',1.20, 'MarkerFaceColor','w', ...
        'HandleVisibility','off');
end

h1 = plot(ax, re1(idx1), im1(idx1), 'x', ...
    'Color', C.blue, 'MarkerSize',5.4, 'LineWidth',1.05);

xline(ax, 0, '--', 'Color', C.gray, 'LineWidth',0.80, 'HandleVisibility','off');

if ~isempty(idx0_rhp)
    text(ax, 0.02, 0.08, 'RHP modes highlighted in red', ...
        'Units','normalized', 'Color', C.red, 'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom', 'FontSize',8, 'Interpreter','latex');
end

re_all = [re0(idx0); re1(idx1); 0];
im_all = [im0(idx0); im1(idx1)];
if isempty(re_all) || isempty(im_all)
    return;
end
xr = max(re_all) - min(re_all);
yr = max(im_all) - min(im_all);
if ~isfinite(xr) || xr <= 0, xr = 1; end
if ~isfinite(yr) || yr <= 0, yr = 1; end
xlim(ax, [min(re_all) - 0.08*xr, max(re_all) + 0.12*xr]);
ylim(ax, [min(im_all) - 0.08*yr, max(im_all) + 0.08*yr]);
end

function eta_min = sampled_eta_min(sysY, w_grid, g)

% Sampled passivity index minimum for Y_c + gI, where Y_c = -Y_dev.
Y = freqresp(sysY, w_grid);
eta = zeros(numel(w_grid),1);
for ii = 1:numel(w_grid)
    Yjw = Y(:,:,ii);
    Sdev = 0.5*(Yjw + Yjw');
    eta(ii) = min(real(eig(g*eye(2) - Sdev)));
end
eta_min = min(eta);
end

function [g_vec, g_scale_used, Acl1, eig1, maxRe1] = enforce_passivated_stability(net, conv, Ssel, g_vec_base, cfg)
% Apply the smallest uniform scaling of the already-passivating g-vector
% that yields a strictly stable passivated closed loop.
g_scale_used = 1.0;
Acl1 = []; eig1 = []; maxRe1 = inf;
for gamma = cfg.g_scale_candidates(:).';
    g_try = gamma * g_vec_base;
    A_try = assemble_global_A(net, conv, Ssel, g_try);
    e_try = eig(full(A_try));
    mr_try = max(real(e_try));
    if mr_try < maxRe1
        Acl1 = A_try; eig1 = e_try; maxRe1 = mr_try; g_vec = g_try; g_scale_used = gamma;
    end
    if (~cfg.enforce_passivated_stability) || (mr_try <= -cfg.stable_margin_target)
        Acl1 = A_try; eig1 = e_try; maxRe1 = mr_try; g_vec = g_try; g_scale_used = gamma;
        return;
    end
end
end

function qv = manual_percentile(x, p)
% Toolbox-free percentile used only for plot zooming.
x = sort(x(:));
if isempty(x)
    qv = NaN; return;
end
p = min(max(p,0),100);
idx = 1 + (numel(x)-1)*(p/100);
i1 = floor(idx);
i2 = ceil(idx);
if i1 == i2
    qv = x(i1);
else
    a = idx - i1;
    qv = (1-a)*x(i1) + a*x(i2);
end
end

function f_mf = compute_midfreq_onset_from_relerr(fHz, rel_err, f_lf, f_hf, thr)
% Empirical onset of the displayed MF regime: first frequency in [f_lf,f_hf]
% for which the relative error falls below thr and remains below thr on most
% of the remaining interval. This is used only for visualization.
idx = find((fHz >= f_lf) & (fHz <= f_hf));
f_mf = NaN;
if isempty(idx), return; end
rr = rel_err(idx); ff = fHz(idx);
for k = 1:numel(idx)
    tail = rr(k:end);
    if rr(k) <= thr && mean(tail <= 1.15*thr) >= 0.75
        f_mf = ff(k);
        return;
    end
end
end

function f_hf = compute_hf_boundary_from_condition(par, w0, w_grid)
% Mid/high-frequency boundary derived from the paper condition
% ||G_I(jw)^{-1} Z_L(jw)||_2 < 1. The boundary is taken as the
% largest frequency on the sampled grid for which the inequality holds.
J = [0 1; -1 0];
I2 = eye(2);
Lf = par.Xf / w0;
metric = false(numel(w_grid),1);
for ii = 1:numel(w_grid)
    s = 1j*w_grid(ii);
    GI = par.Kpc + par.Kic/s;
    ZL = (s*Lf + par.rf)*I2 - (w0*Lf)*J;
    metric(ii) = (norm(ZL,2)/max(abs(GI),1e-12) < 1);
end
idx = find(metric, 1, 'last');
if isempty(idx)
    f_hf = NaN;
else
    f_hf = w_grid(idx)/(2*pi);
end
end
