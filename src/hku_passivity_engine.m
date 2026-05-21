function out = hku_passivity_engine(user)
%HKU_PASSIVITY_ENGINE Core analysis engine for the HKU ECE passivity GUI.
%   OUT = HKU_PASSIVITY_ENGINE(USER) runs the decentralized passivity-based
%   small-signal stability analysis using MATPOWER data and returns all data
%   needed by the GUI.
%
%   This engine is adapted from the no-grounding current-balance case-study
%   scripts. By default, it removes static load admittances, bus GS/BS shunts,
%   and artificial grounding floors from the dynamic network model, while
%   absorbing constant-power loads into local converter equilibrium injections. It supports two placement
%   modes:
%     1) All buses are equipped with converter models.
%     2) All generator buses plus user-selected support buses are equipped
%        with converter models.
%
%   USER fields (most important):
%     .caseSource.type      = 'builtin' or 'file'
%     .caseSource.name      = 'case39' (when builtin)
%     .caseSource.file      = full path to MATPOWER case file (when file)
%     .useAllBuses          = logical
%     .supportBusIDs        = vector of additional support-bus IDs
%     .displayBus           = 0 for auto, else a bus ID
%     .existingGUniform     = scalar existing virtual conductance at all
%                             selected converter buses (default 0)
%
%   Requires MATPOWER and Control System Toolbox.

arguments
    user struct
end

check_dependencies();
define_constants;

cfg = fill_defaults(user);
logLines = strings(0,1);
logLines(end+1) = sprintf('=== Passivity analysis started: %s ===', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

% -------------------------------------------------------------------------
% 1) Power flow
% -------------------------------------------------------------------------
mpopt = mpoption('verbose',0,'out.all',0, ...
                 'pf.alg','NR','pf.tol',1e-10,'pf.nr.max_it',50, ...
                 'pf.enforce_q_lims',0);
[res, stress_used] = run_pf_stressed_safe(cfg.caseSource, cfg.stress, mpopt, cfg.paper_match);
baseMVA = res.baseMVA;
logLines(end+1) = sprintf('Power flow converged. baseMVA=%.3g, x_scale=%.3g, r_scale=%.3g, load_scale_used=%.3g.', ...
    baseMVA, stress_used.x_scale, stress_used.r_scale, stress_used.load_scale_used);

% -------------------------------------------------------------------------
% 2) Converter placement
% -------------------------------------------------------------------------
[conv_bus_ids, conv_bus_idx, Ssel] = select_converter_buses_custom(res, cfg.useAllBuses, cfg.supportBusIDs);
Nc = numel(conv_bus_idx);
gen_bus_ids = unique(res.gen(:, GEN_BUS));
nSupport = sum(~ismember(conv_bus_ids, gen_bus_ids));
logLines(end+1) = sprintf('Converter buses selected: %d total, %d support buses.', Nc, nSupport);
if ~cfg.useAllBuses && ~cfg.netopt.use_static_load_admittance
    error(['No-grounding mode requires all-bus converter placement, because static load ', ...
           'admittances are removed and constant-power loads are absorbed into bus-local ', ...
           'converter equilibrium injections. Use all-bus placement or enable static-load ', ...
           'admittance mode in cfg.netopt.']);
end

% -------------------------------------------------------------------------
% 3) Operating points and network model
% -------------------------------------------------------------------------
op = build_converter_operating_points_selected(res, conv_bus_ids, conv_bus_idx, cfg);
net = build_network_ss_dq(res, cfg.w0, cfg.netopt);
[lam_net, lam_net_min, f_at_min] = network_port_passivity_curve(net, cfg.w0, cfg.w_net_diag, conv_bus_idx); %#ok<ASGLU>
netDiag = network_eigen_diagnostic(net, cfg.w0);
logLines(end+1) = sprintf('Network port passivity diagnostic: min eig(sym(Y_net,port(jw))) = %.3e at %.4g Hz.', lam_net_min, f_at_min);
if ~cfg.netopt.use_static_load_admittance
    logLines(end+1) = sprintf('No-grounding network diagnostic: common-mode pair near +/-j*w0 = %.3e%+.3ej and %.3e%+.3ej; max Re excluding pair = %.3e.', ...
        real(netDiag.lambda_plus), imag(netDiag.lambda_plus), real(netDiag.lambda_minus), imag(netDiag.lambda_minus), netDiag.maxRe_excluding_pair);
elseif lam_net_min <= 0
    logLines(end+1) = "WARNING: loaded-network passivity diagnostic is nonpositive on the sampled grid.";
end

% -------------------------------------------------------------------------
% 4) Converter parameters and local model tuning
% -------------------------------------------------------------------------
Lf = cfg.prm.Xf / cfg.w0;
cfg.prm.Kpc = 2*cfg.prm.zeta_i*cfg.prm.w_ci*Lf - cfg.prm.rf;
cfg.prm.Kic = (cfg.prm.w_ci^2)*Lf;
if cfg.prm.Kpc <= 0
    cfg.prm.Kpc = 0.05;
    logLines(end+1) = "WARNING: computed Kpc<=0; forced Kpc=0.05.";
end

if cfg.useAutoTuneKV
    [prm_tuned, conv0, worstReA] = tune_voltage_loop_all(op, cfg.prm, cfg.w0, cfg.tune, cfg);
else
    prm_fixed = cfg.prm;
    prm_fixed.Kpv = cfg.prm.Kpv;
    prm_fixed.Kiv = cfg.prm.Kiv;
    conv0 = build_all_converters(op, prm_fixed, cfg.w0, cfg);
    worstReA = max([conv0.maxReA]);
    prm_tuned = prm_fixed;
end
logLines(end+1) = sprintf('Voltage-loop setting: Kpv=%.4g, Kiv=%.4g, worst local maxRe(A)=%.3e.', prm_tuned.Kpv, prm_tuned.Kiv, worstReA);

% -------------------------------------------------------------------------
% 5) Current (existing) closed-loop system
% -------------------------------------------------------------------------
g_existing = max(cfg.existingGUniform, 0) * ones(Nc,1);
Acl_cur = assemble_global_A(net, conv0, Ssel, g_existing);
eig_cur = eig(full(Acl_cur));
maxRe_cur = max(real(eig_cur));
logLines(end+1) = sprintf('Current closed-loop spectral margin: max Re(lambda)=%.4e.', maxRe_cur);

% -------------------------------------------------------------------------
% 6) Local passivation thresholds and recommended gains
% -------------------------------------------------------------------------
gmin = zeros(Nc,1);
g0_LF = zeros(Nc,1);
eta_min_current = zeros(Nc,1);
peak_f_gmin = zeros(Nc,1);
for k = 1:Nc
    sysY = conv0(k).sysY_dev;
    g0_LF(k) = g0_from_DC_limit(op(k), conv0(k).par.dq);
    [gmin(k), peak_f_gmin(k)] = compute_gmin_from_ss_refined(sysY, cfg.w_grid, g0_LF(k));
    eta_min_current(k) = sampled_eta_min(sysY, cfg.w_grid, g_existing(k));
end

g_target = max(g_existing, (1+cfg.g_margin_rel)*gmin + cfg.g_margin_abs);
for k = 1:Nc
    eta_k = sampled_eta_min(conv0(k).sysY_dev, cfg.w_grid, g_target(k));
    if eta_k < cfg.eta_margin_abs
        g_target(k) = g_target(k) + (cfg.eta_margin_abs - eta_k);
    end
end

% -------------------------------------------------------------------------
% 7) Recommended passivated system (possibly uniformly scaled if needed)
% -------------------------------------------------------------------------
[g_rec, g_scale_used, Acl_rec, eig_rec, maxRe_rec] = enforce_passivated_stability(net, conv0, Ssel, g_target, cfg);
if g_scale_used > 1
    logLines(end+1) = sprintf('Recommended gains uniformly scaled by %.3g to enforce closed-loop stability.', g_scale_used);
end
logLines(end+1) = sprintf('Recommended closed-loop spectral margin: max Re(lambda)=%.4e.', maxRe_rec);

additional_needed = max(0, g_rec - g_existing);

% -------------------------------------------------------------------------
% 8) Display bus and curves
% -------------------------------------------------------------------------
[kDisplay, kWorstGen, kWorstAll] = choose_display_bus(op, gmin, cfg.displayBus); %#ok<ASGLU>
[lambdaMax_w, fHz] = lambda_max_sym_vs_w(conv0(kDisplay).sysY_dev, cfg.w_grid);
eta_current = g_existing(kDisplay) - lambdaMax_w;
eta_recommended = g_rec(kDisplay) - lambdaMax_w;
[eta_current_min, idx_eta_min] = min(eta_current); %#ok<NASGU>
f_eta_min = fHz(idx_eta_min);

% -------------------------------------------------------------------------
% 9) Time response under a localized disturbance through Delta i_d
% -------------------------------------------------------------------------
t = linspace(0, cfg.time_response.t_end, cfg.time_response.nPoints);
[y_cur, y_rec, u_id, info_td] = simulate_voltage_response_current_disturbance( ...
    Acl_cur, Acl_rec, net, op(kDisplay).bus_idx, t, cfg);

% -------------------------------------------------------------------------
% 10) Output table and summary
% -------------------------------------------------------------------------
resTable = table([op.bus_id].', [op.is_gen_bus].', [op.P0].', [op.Q0].', ...
    g_existing, gmin, g_rec, additional_needed, g0_LF, peak_f_gmin, eta_min_current, ...
    'VariableNames', {'bus_id','is_gen_bus','P0','Q0','g_existing','gmin','g_recommended','g_additional_needed','g0_LF','peak_f_gmin_Hz','eta_min_current'});

summary = struct();
summary.caseName = cfg.caseSource.displayName;
summary.baseMVA = baseMVA;
summary.converterBusIDs = conv_bus_ids(:).';
summary.numConverters = Nc;
summary.numSupportBuses = nSupport;
summary.currentStable = maxRe_cur < 0;
summary.recommendedStable = maxRe_rec < 0;
summary.maxReCurrent = maxRe_cur;
summary.maxReRecommended = maxRe_rec;
summary.displayBus = op(kDisplay).bus_id;
summary.worstBus = op(kWorstAll).bus_id;
summary.worstGenBus = op(kWorstGen).bus_id;
summary.maxAdditionalNeeded = max(additional_needed);
summary.maxAdditionalBus = op(argmax(additional_needed)).bus_id;
summary.maxGmin = max(gmin);
summary.maxGminBus = op(argmax(gmin)).bus_id;
summary.lamNetMin = lam_net_min;
summary.noGroundingNetwork = ~cfg.netopt.use_static_load_admittance;
summary.netLambdaPlus = netDiag.lambda_plus;
summary.netLambdaMinus = netDiag.lambda_minus;
summary.netMaxReAll = netDiag.maxRe_all;
summary.netMaxReExcludingCommon = netDiag.maxRe_excluding_pair;
summary.etaDisplayCurrentMin = min(eta_current);
summary.etaDisplayRecommendedMin = min(eta_recommended);
summary.fStarDisplayHz = f_eta_min;
summary.disturbanceText = sprintf('Localized sinusoidal current burst through \\Delta i_d at bus %d: amp=%.3g pu, f=%.3g Hz, T_b=%.3g s, observed axis=%s.', ...
    op(kDisplay).bus_id, info_td.id_amp, info_td.id_f_Hz, info_td.burst_duration, info_td.axis);

logLines(end+1) = sprintf('Representative bus: %d. Min eta_current=%.4g at %.4g Hz; min eta_recommended=%.4g.', ...
    summary.displayBus, summary.etaDisplayCurrentMin, summary.fStarDisplayHz, summary.etaDisplayRecommendedMin);
logLines(end+1) = summary.disturbanceText;
logLines(end+1) = '=== Passivity analysis finished ===';

out = struct();
out.cfg = cfg;
out.res = res;
out.net = net;
out.op = op;
out.conv = conv0;
out.summary = summary;
out.table = resTable;
out.log = strjoin(cellstr(logLines), newline);
out.cur = struct('Acl', Acl_cur, 'eig', eig_cur, 'g', g_existing, 'eta', eta_current, 'maxRe', maxRe_cur);
out.rec = struct('Acl', Acl_rec, 'eig', eig_rec, 'g', g_rec, 'eta', eta_recommended, 'maxRe', maxRe_rec, ...
                 'additional_needed', additional_needed, 'scale_used', g_scale_used);
out.freq = struct('fHz', fHz, 'lambdaMaxSymYdev', lambdaMax_w, 'fStar', f_eta_min);
out.gains = struct('gmin', gmin, 'g0_LF', g0_LF, 'peak_f_Hz', peak_f_gmin);
out.time = struct('t', t, 'y_current', y_cur, 'y_recommended', y_rec, 'u_id', u_id, 'info', info_td);
out.network = struct('lam_min_curve', lam_net, 'lam_min', lam_net_min, 'f_at_min', f_at_min, ...
                     'eig_diagnostic', netDiag);
end

% ========================================================================
% Helpers
% ========================================================================
function check_dependencies()
if exist('runpf', 'file') ~= 2 || exist('loadcase', 'file') ~= 2 || exist('define_constants', 'file') ~= 2
    error(['MATPOWER was not found on the MATLAB path. ', ...
           'Please add MATPOWER before running the app.']);
end
if exist('ss', 'file') ~= 2 || exist('freqresp', 'file') ~= 2 || exist('lsim', 'file') ~= 2
    error(['Control System Toolbox functions ss/freqresp/lsim were not found. ', ...
           'Please ensure the toolbox is installed.']);
end
end

function cfg = fill_defaults(user)
% Source
if ~isfield(user, 'caseSource'), user.caseSource = struct(); end
if ~isfield(user.caseSource, 'type'), user.caseSource.type = 'builtin'; end
if ~isfield(user.caseSource, 'name'), user.caseSource.name = 'case39'; end
if ~isfield(user.caseSource, 'file'), user.caseSource.file = ''; end
switch lower(user.caseSource.type)
    case 'file'
        cfg.caseSource.loader = string(user.caseSource.file);
        [~, nm, ext] = fileparts(user.caseSource.file);
        cfg.caseSource.displayName = string([nm, ext]);
    otherwise
        cfg.caseSource.loader = string(user.caseSource.name);
        cfg.caseSource.displayName = string(user.caseSource.name);
end

% Base settings
cfg.f0 = getfield_or(user, 'f0', 60);
cfg.w0 = 2*pi*cfg.f0;
cfg.useAllBuses = getfield_or(user, 'useAllBuses', true);
cfg.supportBusIDs = unique(getfield_or(user, 'supportBusIDs', []));
cfg.displayBus = getfield_or(user, 'displayBus', 0);
cfg.existingGUniform = getfield_or(user, 'existingGUniform', 0);

% Stress
cfg.stress.x_scale = getfield_or(getfield_or(user, 'stress', struct()), 'x_scale', 1.5);
cfg.stress.r_scale = getfield_or(getfield_or(user, 'stress', struct()), 'r_scale', 0.7);
cfg.stress.load_scale = getfield_or(getfield_or(user, 'stress', struct()), 'load_scale', 1.0);

% Frequency grids
fGrid = getfield_or(user, 'f_grid_Hz', logspace(-4, 4, 2800));
cfg.f_grid_Hz = fGrid(:).';
cfg.w_grid = 2*pi*cfg.f_grid_Hz;
fNetDiag = getfield_or(user, 'f_net_diag', logspace(-2, 3, 220));
cfg.f_net_diag = fNetDiag(:).';
cfg.w_net_diag = 2*pi*cfg.f_net_diag;

% Paper matching
pm = getfield_or(user, 'paper_match', struct());
cfg.paper_match.remove_taps_and_shifts = getfield_or(pm, 'remove_taps_and_shifts', true);
cfg.paper_match.remove_bus_shunts = getfield_or(pm, 'remove_bus_shunts', true);
cfg.paper_match.gs_floor_pu = getfield_or(pm, 'gs_floor_pu', 0);

% Margins
cfg.g_margin_rel = getfield_or(user, 'g_margin_rel', 0.02);
cfg.g_margin_abs = getfield_or(user, 'g_margin_abs', 1e-4);
cfg.eta_margin_abs = getfield_or(user, 'eta_margin_abs', 1e-7);
cfg.enforce_passivated_stability = getfield_or(user, 'enforce_passivated_stability', true);
cfg.stable_margin_target = getfield_or(user, 'stable_margin_target', 1e-5);
cfg.g_scale_candidates = getfield_or(user, 'g_scale_candidates', [1.0 1.01 1.02 1.05 1.10 1.20 1.50 2 3]);

% Network options
netopt = getfield_or(user, 'netopt', struct());
cfg.netopt.g_eps = getfield_or(netopt, 'g_eps', 0);
cfg.netopt.c_min = getfield_or(netopt, 'c_min', 1e-6);
cfg.netopt.assert_positive_G = getfield_or(netopt, 'assert_positive_G', false);
cfg.netopt.use_static_load_admittance = getfield_or(netopt, 'use_static_load_admittance', false);

% Converter parameters
prm = getfield_or(user, 'prm', struct());
cfg.prm.Xf = getfield_or(prm, 'Xf', 0.15);
cfg.prm.rf = getfield_or(prm, 'rf', 0.01);
cfg.prm.w_ci = 2*pi*getfield_or(prm, 'w_ci_Hz', 300);
cfg.prm.zeta_i = getfield_or(prm, 'zeta_i', 1.0);
cfg.prm.dp = getfield_or(prm, 'dp', 5.0);
cfg.prm.dq = getfield_or(prm, 'dq', 0.01);
cfg.prm.tau_p = getfield_or(prm, 'tau_p', 0.05);
cfg.prm.tau_q = getfield_or(prm, 'tau_q', 0.05);
cfg.prm.Kpv = getfield_or(prm, 'Kpv', 1.5);
cfg.prm.Kiv = getfield_or(prm, 'Kiv', 0.2);

cfg.useAutoTuneKV = getfield_or(user, 'useAutoTuneKV', true);

% Tuning grid
cfg.tune.Kpv_list = getfield_or(getfield_or(user, 'tune', struct()), 'Kpv_list', [0.05 0.10 0.20 0.30 0.50 0.80 1.0 1.5 2.0 3.0 5.0]);
cfg.tune.Kiv_list = getfield_or(getfield_or(user, 'tune', struct()), 'Kiv_list', [1e-3 2e-3 5e-3 0.01 0.02 0.05 0.10 0.20 0.50 1.0 2.0 5.0 10.0]);
cfg.tune.stab_margin = getfield_or(getfield_or(user, 'tune', struct()), 'stab_margin', 2e-3);
cfg.tune.auto_expand = true;
cfg.tune.allow_dp_relax = false;

% Support profile
sp = getfield_or(user, 'support_profile', struct());
cfg.support_profile.enable = getfield_or(sp, 'enable', true);
cfg.support_profile.dp_non_gen = getfield_or(sp, 'dp_non_gen', 0.25);
cfg.support_profile.dq_non_gen = getfield_or(sp, 'dq_non_gen', 0.05);
cfg.support_profile.tau_p_non_gen = getfield_or(sp, 'tau_p_non_gen', 0.20);
cfg.support_profile.tau_q_non_gen = getfield_or(sp, 'tau_q_non_gen', 0.20);

% Time response
tr = getfield_or(user, 'time_response', struct());
cfg.time_response.axis = getfield_or(tr, 'axis', 'q');
cfg.time_response.id_amp = getfield_or(tr, 'id_amp', 1e-3);
cfg.time_response.id_f_Hz = getfield_or(tr, 'id_f_Hz', 3.0);
cfg.time_response.burst_duration = getfield_or(tr, 'burst_duration', 0.8);
cfg.time_response.t_end = getfield_or(tr, 't_end', 1.2);
cfg.time_response.nPoints = round(getfield_or(tr, 'nPoints', 2400));
end

function val = getfield_or(s, field, default)
if isstruct(s) && isfield(s, field)
    val = s.(field);
else
    val = default;
end
end

function [res, stress_used] = run_pf_stressed_safe(caseSource, stress, mpopt, paper_match)
define_constants;
mpc0 = loadcase(char(caseSource.loader));

mpc0.branch(:, BR_R) = mpc0.branch(:, BR_R) * stress.r_scale;
mpc0.branch(:, BR_X) = mpc0.branch(:, BR_X) * stress.x_scale;

if paper_match.remove_taps_and_shifts
    mpc0.branch(:, SHIFT) = 0;
    ix_tap = find(mpc0.branch(:, TAP) ~= 0);
    mpc0.branch(ix_tap, TAP) = 1;
end
if isfield(paper_match, 'remove_bus_shunts') && paper_match.remove_bus_shunts
    mpc0.bus(:, GS) = 0;
    mpc0.bus(:, BS) = 0;
end
if paper_match.gs_floor_pu > 0
    gs_floor_MW = paper_match.gs_floor_pu * mpc0.baseMVA;
    mpc0.bus(:, GS) = max(mpc0.bus(:, GS), gs_floor_MW);
end

load_scale = stress.load_scale;
for k = 1:12
    mpc = mpc0;
    mpc.bus(:, PD) = mpc0.bus(:, PD) * load_scale;
    mpc.bus(:, QD) = mpc0.bus(:, QD) * load_scale;
    mpc.gen(:, PG) = mpc0.gen(:, PG) * load_scale;
    res = runpf(mpc, mpopt);
    if res.success
        stress_used = stress;
        stress_used.load_scale_used = load_scale;
        stress_used.remove_taps_and_shifts = paper_match.remove_taps_and_shifts;
        stress_used.gs_floor_pu = paper_match.gs_floor_pu;
        stress_used.remove_bus_shunts = isfield(paper_match, 'remove_bus_shunts') && paper_match.remove_bus_shunts;
        return;
    end
    load_scale = 0.9 * load_scale;
end
error('Power flow failed to converge even after load backoff.');
end

function [conv_bus_ids, conv_bus_idx, S] = select_converter_buses_custom(res, useAllBuses, supportBusIDs)
define_constants;
bus_ids = res.bus(:, BUS_I);
gen_bus_ids = unique(res.gen(:, GEN_BUS));
if useAllBuses
    conv_bus_ids = bus_ids;
else
    conv_bus_ids = unique([gen_bus_ids(:); supportBusIDs(:)]);
end
[tf, conv_bus_idx] = ismember(conv_bus_ids, bus_ids);
if any(~tf)
    error('At least one selected converter bus was not found in the case file.');
end
Nc = numel(conv_bus_idx);
N = numel(bus_ids);
S = sparse(2*N, 2*Nc);
for k = 1:Nc
    i = conv_bus_idx(k);
    S(2*i-1, 2*k-1) = 1;
    S(2*i,   2*k  ) = 1;
end
end

function op = build_converter_operating_points_selected(res, conv_bus_ids, conv_bus_idx, cfg)
define_constants;
baseMVA = res.baseMVA;
bus_ids = res.bus(:, BUS_I);
N = size(res.bus,1);
Vm = res.bus(:, VM);
Va = res.bus(:, VA) * pi/180;
V  = Vm .* exp(1j*Va);

gen_bus_idx = zeros(size(res.gen,1),1);
for gg = 1:size(res.gen,1)
    [~, bi] = ismember(res.gen(gg, GEN_BUS), bus_ids);
    gen_bus_idx(gg) = bi;
end
Pg_bus = accumarray(gen_bus_idx, res.gen(:, PG)/baseMVA, [N 1], @sum, 0);
Qg_bus = accumarray(gen_bus_idx, res.gen(:, QG)/baseMVA, [N 1], @sum, 0);
gen_bus_id_set = unique(res.gen(:, GEN_BUS));
Pload_bus = res.bus(:, PD)/baseMVA;
Qload_bus = res.bus(:, QD)/baseMVA;

Nc = numel(conv_bus_idx);
op = repmat(struct('v0',[],'i0',[],'delta0',0,'E0',0,'P0',0,'Q0',0, ...
                   'bus_idx',0,'bus_id',0,'is_gen_bus',false), Nc, 1);
for k = 1:Nc
    bi = conv_bus_idx(k);
    Vk = V(bi);
    isGen = ismember(conv_bus_ids(k), gen_bus_id_set);
    if cfg.netopt.use_static_load_admittance
        % Loaded-network mode: loads are represented as static network admittances.
        if isGen
            Sk = Pg_bus(bi) + 1j*Qg_bus(bi);
        else
            Sk = 0;
        end
    else
        % No-grounding current-balance mode: static load admittances are removed
        % from the dynamic network, so constant-power loads are absorbed into
        % the local converter equilibrium injection.
        Sk = (Pg_bus(bi) - Pload_bus(bi)) + 1j*(Qg_bus(bi) - Qload_bus(bi));
    end
    Ik = conj(Sk / Vk);
    op(k).v0 = [real(Vk); imag(Vk)];
    op(k).i0 = [real(Ik); imag(Ik)];
    op(k).delta0 = angle(Vk);
    op(k).E0 = abs(Vk);
    op(k).P0 = real(Sk);
    op(k).Q0 = imag(Sk);
    op(k).bus_idx = bi;
    op(k).bus_id = conv_bus_ids(k);
    op(k).is_gen_bus = isGen;
end
end

function prm_vec = make_buswise_params(op, prm_base, cfg)
Nc = numel(op);
prm_vec = repmat(prm_base, Nc, 1);
if cfg.support_profile.enable
    for k = 1:Nc
        if ~op(k).is_gen_bus
            prm_vec(k).dp = cfg.support_profile.dp_non_gen;
            prm_vec(k).dq = cfg.support_profile.dq_non_gen;
            prm_vec(k).tau_p = cfg.support_profile.tau_p_non_gen;
            prm_vec(k).tau_q = cfg.support_profile.tau_q_non_gen;
        end
    end
end
end

function [prm_out, conv, worstReA] = tune_voltage_loop_all(op, prm_in, w0, tune, cfg)
userKpv = sort(unique(tune.Kpv_list(:).'));
userKiv = sort(unique(tune.Kiv_list(:).'));
search_sets = {};
search_sets{end+1} = struct('Kpv_list', userKpv, 'Kiv_list', userKiv, 'dp_scale', 1.0);
extKpv = sort(unique([userKpv, 0.8 1.0 1.2 1.5 2 3 5 8 10 15 20]));
extKiv = sort(unique([userKiv, 1e-3 2e-3 5e-3 0.01 0.02 0.05 0.1 0.2 0.5 1 2 5 10 20]));
search_sets{end+1} = struct('Kpv_list', extKpv, 'Kiv_list', extKiv, 'dp_scale', 1.0);

bestWorst = Inf; prm_out = prm_in; conv = []; worstReA = Inf;
for ss = 1:numel(search_sets)
    sset = search_sets{ss};
    prm_base = prm_in; prm_base.dp = prm_in.dp * sset.dp_scale;
    for Kpv = sset.Kpv_list
        for Kiv = sset.Kiv_list
            prm = prm_base; prm.Kpv = Kpv; prm.Kiv = Kiv;
            conv_try = build_all_converters(op, prm, w0, cfg);
            worst = max([conv_try.maxReA]);
            if worst < bestWorst
                bestWorst = worst; prm_out = prm; conv = conv_try; worstReA = worst;
            end
            if worst <= -tune.stab_margin
                prm_out = prm; conv = conv_try; worstReA = worst; return;
            end
        end
    end
end
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
J = [0 1; -1 0]; I2 = eye(2);
Lf = prm.Xf / w0; rf = prm.rf;
v0 = op.v0(:); i0 = op.i0(:); delta0 = op.delta0; E0 = op.E0;
ucc0 = v0 + rf*i0 - (w0*Lf)*J*i0;
xiV0 = i0; xiI0 = ucc0;
P_E = [cos(delta0); sin(delta0)];
P_d = [-E0*sin(delta0); E0*cos(delta0)];
idx_i=1:2; idx_xiI=3:4; idx_xiV=5:6; idx_del=7; idx_xp=8; idx_xq=9;
A = zeros(9,9); B = zeros(9,2);
A(idx_i, idx_i)   = w0*J - ((rf + prm.Kpc)/Lf)*I2;
A(idx_i, idx_xiV) = (prm.Kpc/Lf)*I2;
A(idx_i, idx_xiI) = (1/Lf)*I2;
A(idx_i, idx_del) = (prm.Kpc*prm.Kpv/Lf) * P_d;
A(idx_i, idx_xq)  = (-prm.Kpc*prm.Kpv*prm.dq/Lf) * P_E;
B(idx_i, :)       = -((prm.Kpc*prm.Kpv + 1)/Lf)*I2;
A(idx_xiI, idx_i)   = -prm.Kic*I2;
A(idx_xiI, idx_xiV) =  prm.Kic*I2;
A(idx_xiI, idx_del) = (prm.Kic*prm.Kpv) * P_d;
A(idx_xiI, idx_xq)  = (-prm.Kic*prm.Kpv*prm.dq) * P_E;
A(idx_xiI, idx_xp)  = (prm.dp) * (J*xiI0);
B(idx_xiI, :)       = -(prm.Kic*prm.Kpv)*I2;
A(idx_xiV, idx_del) = (prm.Kiv) * P_d;
A(idx_xiV, idx_xq)  = (-prm.Kiv*prm.dq) * P_E;
A(idx_xiV, idx_xp)  = (prm.dp) * (J*xiV0);
B(idx_xiV, :)       = -(prm.Kiv)*I2;
A(idx_del, idx_xp) = -prm.dp;
A(idx_xp, idx_i)  = (1/prm.tau_p) * (v0.');
A(idx_xp, idx_xp) = -1/prm.tau_p;
B(idx_xp, :)      = (1/prm.tau_p) * (i0.');
A(idx_xq, idx_i)  = (1/prm.tau_q) * (-v0.'*J);
A(idx_xq, idx_xq) = -1/prm.tau_q;
B(idx_xq, :)      = (1/prm.tau_q) * (i0.'*J);
C = [I2 zeros(2,7)]; D = zeros(2,2);
maxReA = max(real(eig(A)));
end

function net = build_network_ss_dq(res, w0, opts)
define_constants;
bus = res.bus; branch_all = res.branch; baseMVA = res.baseMVA;
branch = branch_all(branch_all(:, BR_STATUS) == 1, :);
bus_ids = bus(:, BUS_I); N = size(bus,1); M = size(branch,1);
J = sparse([0 1; -1 0]); I2 = speye(2);
[~, f_idx] = ismember(branch(:, F_BUS), bus_ids); [~, t_idx] = ismember(branch(:, T_BUS), bus_ids);
Bcap = zeros(N,1);
for k = 1:M
    b = branch(k, BR_B); if b == 0, continue; end
    tau = branch(k, TAP); if tau == 0, tau = 1; end
    Bcap(f_idx(k)) = Bcap(f_idx(k)) + 0.5*b/(tau^2);
    Bcap(t_idx(k)) = Bcap(t_idx(k)) + 0.5*b;
end
Bbus = bus(:, BS)/baseMVA;
Bcap = Bcap + max(0, Bbus);
c = Bcap / w0; c(c < opts.c_min) = opts.c_min;
Vm = bus(:, VM); V2 = max(Vm.^2, 1e-8);
if isfield(opts, 'use_static_load_admittance') && opts.use_static_load_admittance
    Ppu = bus(:, PD)/baseMVA; Qpu = bus(:, QD)/baseMVA;
    G = Ppu ./ V2; B = -Qpu ./ V2;
    G = G + bus(:, GS)/baseMVA;
    B = B + min(0, Bbus);
    G = G + opts.g_eps;
else
    G = zeros(N,1);
    B = zeros(N,1);
end
YL = spalloc(2*N, 2*N, 4*N);
for i = 1:N
    blk = G(i)*I2 - B(i)*J;
    YL(2*i-1:2*i, 2*i-1:2*i) = blk;
end
E_v = spalloc(2*M, 2*N, 8*M); E_i = spalloc(2*N, 2*M, 8*M);
for k = 1:M
    tau = branch(k, TAP); if tau == 0, tau = 1; end
    phi = branch(k, SHIFT)*pi/180;
    Rm = [cos(phi) sin(phi); -sin(phi) cos(phi)];
    A = (1/tau)*Rm; Bm = A.';
    rf = 2*f_idx(k)-1:2*f_idx(k); rt = 2*t_idx(k)-1:2*t_idx(k); rk = 2*k-1:2*k;
    E_v(rk, rf) = A; E_v(rk, rt) = -eye(2);
    E_i(rf, rk) = Bm; E_i(rt, rk) = -eye(2);
end
r = branch(:, BR_R); x = branch(:, BR_X); L = x/w0; L(L<1e-10)=1e-10;
Cinv = kron(spdiags(1./c,0,N,N), I2);
Linv = kron(spdiags(1./L,0,M,M), I2);
RovL = kron(spdiags(r./L,0,M,M), I2);
JN = kron(speye(N), J); JM = kron(speye(M), J);
A_vv = w0*JN - Cinv*YL; A_vi = -Cinv*E_i; B_v = Cinv;
A_iv = Linv*E_v; A_ii = w0*JM - RovL;
net.N=N; net.M=M; net.A_vv=A_vv; net.A_vi=A_vi; net.B_v=B_v; net.A_iv=A_iv; net.A_ii=A_ii;
net.c=c; net.YL=YL; net.Ev=E_v; net.Ei=E_i; net.r=r; net.L=L;
net.use_static_load_admittance = isfield(opts, 'use_static_load_admittance') && opts.use_static_load_admittance;
end

function [lam_curve, lam_min, f_at_min] = network_port_passivity_curve(net, w0, w_grid, port_bus_idx)
N = net.N; I2 = eye(2); J = [0 1; -1 0];
port_dof = reshape([2*port_bus_idx(:)-1, 2*port_bus_idx(:)].',1,[]);
all_dof = 1:(2*N); int_dof = setdiff(all_dof, port_dof);
Cblk = kron(diag(net.c), I2); CJblk = kron(diag(net.c), J);
Lblk = diag(net.L); Rblk = diag(net.r);
lam_curve = zeros(numel(w_grid),1);
for ii = 1:numel(w_grid)
    w = w_grid(ii); s = 1j*w;
    Z = kron(s*Lblk + Rblk, I2) - w0*kron(Lblk, J);
    Y_lines = net.Ei * (Z \ net.Ev);
    Y_tot = s*Cblk - w0*CJblk + Y_lines + net.YL;
    YPP = Y_tot(port_dof, port_dof);
    if isempty(int_dof)
        Yport = YPP;
    else
        YPQ = Y_tot(port_dof, int_dof); YQP = Y_tot(int_dof, port_dof); YQQ = Y_tot(int_dof, int_dof);
        Yport = YPP - YPQ * (YQQ \ YQP);
    end
    S = 0.5*(Yport + Yport');
    lam_curve(ii) = min(real(eig(full(S))));
end
[lam_min, idx] = min(lam_curve); f_at_min = w_grid(idx)/(2*pi);
end

function diag = network_eigen_diagnostic(net, w0)
Anet = [net.A_vv, net.A_vi; net.A_iv, net.A_ii];
e = eig(full(Anet));
[~, idx_plus] = min(abs(e - 1j*w0));
[~, idx_minus] = min(abs(e + 1j*w0));
mask = false(size(e));
mask(idx_plus) = true; mask(idx_minus) = true;
rem = e(~mask);
diag = struct();
diag.eig = e;
diag.lambda_plus = e(idx_plus);
diag.lambda_minus = e(idx_minus);
diag.maxRe_all = max(real(e));
if isempty(rem)
    diag.maxRe_excluding_pair = NaN;
else
    diag.maxRe_excluding_pair = max(real(rem));
end
end

function Acl = assemble_global_A(net, conv, S, g_vec)
N = net.N; M = net.M; Nc = numel(conv); %#ok<NASGU>
nx = 9*Nc;
A_c = sparse(nx, nx); B_c = sparse(nx, 2*Nc); C_c = sparse(2*Nc, nx);
for k = 1:Nc
    ix = (9*(k-1)+1):(9*k); iu = (2*(k-1)+1):(2*k);
    A_c(ix,ix) = conv(k).A; B_c(ix,iu) = conv(k).B; C_c(iu,ix) = conv(k).C;
end
D_c = kron(spdiags(-g_vec,0,Nc,Nc), speye(2));
A_vv_eff = net.A_vv + (net.B_v * S) * D_c * S.';
A_vx = (net.B_v * S) * C_c;
A_xv = B_c * S.';
Acl = [A_vv_eff, net.A_vi, A_vx; net.A_iv, net.A_ii, sparse(2*M, 9*Nc); A_xv, sparse(9*Nc, 2*M), A_c];
end

function [gmin, fpeak_Hz] = compute_gmin_from_ss_refined(sysY, w_grid, g0_LF)
[lambdaMax, ~] = lambda_max_curve(sysY, w_grid);
[lam_peak, idx_peak] = max(lambdaMax); w_peak = w_grid(idx_peak);
w_lo = max(w_grid(1), w_peak/10); w_hi = min(w_grid(end), w_peak*10);
if w_hi > w_lo
    w_ref = unique([w_grid(:); logspace(log10(w_lo), log10(w_hi), 1200).']);
    [lambdaMax_ref, ~] = lambda_max_curve(sysY, w_ref);
    [lam_peak, idx_peak] = max(lambdaMax_ref); w_peak = w_ref(idx_peak);
end
gmin = max([0; g0_LF; lam_peak]); fpeak_Hz = w_peak/(2*pi);
end

function [lambdaMax, fHz] = lambda_max_curve(sysY, w_grid)
Y = freqresp(sysY, w_grid); lambdaMax = zeros(numel(w_grid),1);
for ii = 1:numel(w_grid)
    S = 0.5*(Y(:,:,ii) + Y(:,:,ii)');
    lambdaMax(ii) = max(real(eig(S)));
end
fHz = w_grid/(2*pi);
end

function [lambdaMax_w, fHz] = lambda_max_sym_vs_w(sysY, w_grid)
[lambdaMax_w, fHz] = lambda_max_curve(sysY, w_grid);
end

function g0 = g0_from_DC_limit(op, dq)
v0 = op.v0(:); i0 = op.i0(:); delta = atan2(v0(2), v0(1));
Q = [cos(-delta) -sin(-delta); sin(-delta) cos(-delta)];
v_al = Q*v0; i_al = Q*i0; v = v_al(1); id = i_al(1); iq = i_al(2);
g0 = sqrt((id/v)^2 + (iq/v - 1/(2*dq*v))^2);
end

function [y_cur, y_rec, u, info] = simulate_voltage_response_current_disturbance(Acur, Arec, net, bus_idx, t, cfg)
nTot = size(Acur,1); N = net.N; extra = nTot - 2*N;
switch lower(cfg.time_response.axis)
    case 'd'
        busvec = zeros(2*N,1); busvec(2*bus_idx-1) = 1;
        Csel = sparse(1, nTot); Csel(1, 2*bus_idx-1) = 1;
    otherwise
        busvec = zeros(2*N,1); busvec(2*bus_idx) = 1;
        Csel = sparse(1, nTot); Csel(1, 2*bus_idx) = 1;
end
Bext = [net.B_v * busvec; zeros(extra,1)];
info.id_amp = cfg.time_response.id_amp;
info.id_f_Hz = cfg.time_response.id_f_Hz;
info.burst_duration = cfg.time_response.burst_duration;
info.axis = cfg.time_response.axis;
info.bus_id = bus_idx;
tcol = t(:); u = zeros(size(tcol)); Tb = info.burst_duration; idx = (tcol <= Tb);
if any(idx)
    tau = tcol(idx);
    env = 0.5*(1 - cos(2*pi*tau/Tb));
    u(idx) = info.id_amp * sin(2*pi*info.id_f_Hz*tau) .* env;
end
if ~isempty(u), u(1) = 0; end
sysCur = ss(full(Acur), full(Bext), full(Csel), 0);
sysRec = ss(full(Arec), full(Bext), full(Csel), 0);
y_cur = lsim(sysCur, u, t, zeros(nTot,1));
y_rec = lsim(sysRec, u, t, zeros(nTot,1));
y_cur = y_cur(:); y_rec = y_rec(:);
end

function eta_min = sampled_eta_min(sysY, w_grid, g)
Y = freqresp(sysY, w_grid); eta = zeros(numel(w_grid),1);
for ii = 1:numel(w_grid)
    Sdev = 0.5*(Y(:,:,ii) + Y(:,:,ii)');
    eta(ii) = min(real(eig(g*eye(2) - Sdev)));
end
eta_min = min(eta);
end

function [g_vec, g_scale_used, Acl1, eig1, maxRe1] = enforce_passivated_stability(net, conv, Ssel, g_vec_base, cfg)
g_scale_used = 1.0; Acl1 = []; eig1 = []; maxRe1 = inf; g_vec = g_vec_base;
for gamma = cfg.g_scale_candidates(:).'
    g_try = gamma * g_vec_base;
    A_try = assemble_global_A(net, conv, Ssel, g_try);
    e_try = eig(full(A_try)); mr_try = max(real(e_try));
    if mr_try < maxRe1
        Acl1 = A_try; eig1 = e_try; maxRe1 = mr_try; g_vec = g_try; g_scale_used = gamma;
    end
    if (~cfg.enforce_passivated_stability) || (mr_try <= -cfg.stable_margin_target)
        Acl1 = A_try; eig1 = e_try; maxRe1 = mr_try; g_vec = g_try; g_scale_used = gamma; return;
    end
end
end

function [kDisplay, kWorstGen, kWorstAll] = choose_display_bus(op, gmin, forcedBus)
[~, kWorstAll] = max(gmin);
genMask = [op.is_gen_bus];
if any(genMask)
    idxGen = find(genMask); [~, ixGen] = max(gmin(idxGen)); kWorstGen = idxGen(ixGen);
else
    kWorstGen = kWorstAll;
end
if forcedBus > 0
    kDisplay = find([op.bus_id] == forcedBus, 1);
    if isempty(kDisplay)
        kDisplay = kWorstGen;
    end
else
    kDisplay = kWorstGen;
end
end

function k = argmax(x)
[~, k] = max(x);
end
