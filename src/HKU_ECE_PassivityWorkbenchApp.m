
classdef HKU_ECE_PassivityWorkbenchApp < matlab.apps.AppBase
    % HKU_ECE_PassivityWorkbenchApp
    % Class-based MATLAB app for decentralized passivity-based small-signal
    % stability analysis of converter-dominated power systems.
    %
    % Notes:
    %   - Uses hku_passivity_engine.m as the numerical backend.
    %   - The passivity-index panel always shows the LIMITING converter
    %     (largest g_min), which is the most relevant bus for the paper.
    %   - The time-response panel uses the selected display bus if a
    %     positive bus number is entered; otherwise it uses the limiting bus.

    properties (Access = public)
        UIFigure matlab.ui.Figure
    end

    properties (Access = private)
        AppRoot string
        PreviewCase
        PreviewMeta struct
        LastResult

        % Top-level layout
        MainGrid matlab.ui.container.GridLayout
        BannerPanel matlab.ui.container.Panel
        ContentGrid matlab.ui.container.GridLayout
        SetupPanel matlab.ui.container.Panel
        WorkspaceTabs matlab.ui.container.TabGroup

        % Banner
        BannerGrid matlab.ui.container.GridLayout
        BrandPanel matlab.ui.container.Panel
        BrandGrid matlab.ui.container.GridLayout
        LogoImage matlab.ui.control.Image
        TitleLabel matlab.ui.control.Label
        AuthorsLabel matlab.ui.control.Label
        DeptLabel matlab.ui.control.Label
        CurrentPanel matlab.ui.container.Panel
        RecommendedPanel matlab.ui.container.Panel
        CurrentLamp matlab.ui.control.Lamp
        RecommendedLamp matlab.ui.control.Lamp
        CurrentStatus matlab.ui.control.Label
        RecommendedStatus matlab.ui.control.Label

        % Setup area
        SetupGrid matlab.ui.container.GridLayout
        SetupTabs matlab.ui.container.TabGroup
        BasicTab matlab.ui.container.Tab
        AdvancedTab matlab.ui.container.Tab
        ButtonGrid matlab.ui.container.GridLayout
        BasicScroll matlab.ui.container.Panel
        AdvancedScroll matlab.ui.container.Panel

        % Workspace
        DashboardTab matlab.ui.container.Tab
        ResultsTab matlab.ui.container.Tab
        DashboardGrid matlab.ui.container.GridLayout
        SummaryPanel matlab.ui.container.Panel
        SummaryText matlab.ui.control.TextArea
        PlotGrid matlab.ui.container.GridLayout
        AxesA matlab.ui.control.UIAxes
        AxesB matlab.ui.control.UIAxes
        AxesC matlab.ui.control.UIAxes
        AxesD matlab.ui.control.UIAxes
        ResultsGrid matlab.ui.container.GridLayout
        ResultsSummaryLabel matlab.ui.control.Label
        ResultsTable matlab.ui.control.Table

        % Controls - Basic
        SourceType matlab.ui.control.DropDown
        CaseField matlab.ui.control.EditField
        BrowseBtn matlab.ui.control.Button
        CasePreviewText matlab.ui.control.TextArea
        PlacementMode matlab.ui.control.DropDown
        SupportBusField matlab.ui.control.EditField
        DisplayBusField matlab.ui.control.NumericEditField
        XScale matlab.ui.control.NumericEditField
        RScale matlab.ui.control.NumericEditField
        LoadScale matlab.ui.control.NumericEditField
        GSFloor matlab.ui.control.NumericEditField
        PreviewBtn matlab.ui.control.Button
        RunBtn matlab.ui.control.Button
        ExportBtn matlab.ui.control.Button
        ResetBtn matlab.ui.control.Button

        % Controls - Advanced
        DistAxis matlab.ui.control.DropDown
        IdAmp matlab.ui.control.NumericEditField
        IdFreq matlab.ui.control.NumericEditField
        BurstDuration matlab.ui.control.NumericEditField
        Xf matlab.ui.control.NumericEditField
        Rf matlab.ui.control.NumericEditField
        WciHz matlab.ui.control.NumericEditField
        ZetaI matlab.ui.control.NumericEditField
        Dp matlab.ui.control.NumericEditField
        Dq matlab.ui.control.NumericEditField
        TauP matlab.ui.control.NumericEditField
        TauQ matlab.ui.control.NumericEditField
        AutoTune matlab.ui.control.CheckBox
        Kpv matlab.ui.control.NumericEditField
        Kiv matlab.ui.control.NumericEditField
        SupportProfile matlab.ui.control.CheckBox
        SupportDp matlab.ui.control.NumericEditField
        SupportDq matlab.ui.control.NumericEditField
        SupportTauP matlab.ui.control.NumericEditField
        SupportTauQ matlab.ui.control.NumericEditField
        ExistingG matlab.ui.control.NumericEditField
        GMarginRel matlab.ui.control.NumericEditField
        GMarginAbs matlab.ui.control.NumericEditField
        EtaMargin matlab.ui.control.NumericEditField
        StabMargin matlab.ui.control.NumericEditField
        AdvText matlab.ui.control.TextArea
    end

    methods (Access = public)
        function app = HKU_ECE_PassivityWorkbenchApp
            app.AppRoot = string(fileparts(mfilename('fullpath')));
            app.PreviewCase = [];
            app.LastResult = [];
            app.PreviewMeta = struct('sourceType',"builtin",'caseName',"case39",'caseFile',"");
            app.createComponents();
            registerApp(app, app.UIFigure);
            app.resetDefaults();
            app.previewCurrentCase();
            if nargout == 0
                clear app
            end
        end

        function delete(app)
            try
                if isvalid(app.UIFigure)
                    delete(app.UIFigure);
                end
            catch
            end
        end
    end

    methods (Access = private)
        function createComponents(app)
            bg = [0.97 0.98 0.99];
            navy = [0.08 0.18 0.36];
            grayText = [0.32 0.36 0.43];

            app.UIFigure = uifigure('Name','HKU ECE Passivity-Based Stability Workbench', ...
                'Color', bg, 'Position', [40 40 1620 960], 'AutoResizeChildren','off');
            app.UIFigure.SizeChangedFcn = @(~,~)app.updateResponsiveLayout();

            app.MainGrid = uigridlayout(app.UIFigure,[2 1]);
            app.MainGrid.RowHeight = {112, '1x'};
            app.MainGrid.Padding = [8 8 8 8];
            app.MainGrid.RowSpacing = 10;
            app.MainGrid.BackgroundColor = bg;

            % ================= Banner =================
            app.BannerPanel = uipanel(app.MainGrid,'BorderType','none','BackgroundColor','white');
            app.BannerPanel.Layout.Row = 1; app.BannerPanel.Layout.Column = 1;

            app.BannerGrid = uigridlayout(app.BannerPanel,[1 3]);
            app.BannerGrid.ColumnWidth = {'1x', 145, 145};
            app.BannerGrid.RowHeight = {'1x'};
            app.BannerGrid.Padding = [12 10 12 10];
            app.BannerGrid.ColumnSpacing = 12;
            app.BannerGrid.BackgroundColor = 'white';

            app.BrandPanel = uipanel(app.BannerGrid,'BorderType','none','BackgroundColor','white');
            app.BrandPanel.Layout.Row = 1; app.BrandPanel.Layout.Column = 1;
            app.BrandGrid = uigridlayout(app.BrandPanel,[3 2]);
            app.BrandGrid.ColumnWidth = {220, '1x'};
            app.BrandGrid.RowHeight = {36, 24, 22};
            app.BrandGrid.Padding = [0 0 0 0];
            app.BrandGrid.ColumnSpacing = 10;
            app.BrandGrid.RowSpacing = 2;
            app.BrandGrid.BackgroundColor = 'white';

            logoFile = fullfile(app.AppRoot, 'hku_logo_clean_white.png');
            if exist(logoFile,'file') ~= 2
                logoFile = fullfile(fileparts(app.AppRoot), 'assets', 'hku_logo_clean_white.png');
            end
            if exist(logoFile,'file') ~= 2
                alt1 = fullfile(app.AppRoot, 'hku_official_logo_english.png');
                alt2 = fullfile(app.AppRoot, 'hku_official_logo_english.jpg');
                alt3 = fullfile(app.AppRoot, 'hku_ece_placeholder.png');
                if exist(alt1,'file') == 2, logoFile = alt1;
                elseif exist(alt2,'file') == 2, logoFile = alt2;
                elseif exist(alt3,'file') == 2, logoFile = alt3;
                else, logoFile = "";
                end
            end

            app.LogoImage = uiimage(app.BrandGrid,'ScaleMethod','fit');
            app.LogoImage.Layout.Row = [1 3];
            app.LogoImage.Layout.Column = 1;
            if strlength(string(logoFile)) > 0
                app.LogoImage.ImageSource = char(logoFile);
            end

            app.TitleLabel = uilabel(app.BrandGrid, ...
                'Text','Passivity-Based Stability Workbench', ...
                'FontName','Times New Roman','FontWeight','bold','FontSize',25, ...
                'FontColor',navy,'HorizontalAlignment','left');
            app.TitleLabel.Layout.Row = 1; app.TitleLabel.Layout.Column = 2;

            app.AuthorsLabel = uilabel(app.BrandGrid, ...
                'Text','Yongheng Wang, Xiemin Mo, and Tao Liu', ...
                'FontName','Arial','FontWeight','bold','FontSize',11, ...
                'FontColor',[0.15 0.15 0.18], 'HorizontalAlignment','left');
            app.AuthorsLabel.Layout.Row = 2; app.AuthorsLabel.Layout.Column = 2;

            app.DeptLabel = uilabel(app.BrandGrid, ...
                'Text','HKU ECE | Current-Balance No-Grounding Release', ...
                'FontName','Arial','FontSize',10.5,'FontColor',grayText,'HorizontalAlignment','left');
            app.DeptLabel.Layout.Row = 3; app.DeptLabel.Layout.Column = 2;

            % status cards
            app.CurrentPanel = uipanel(app.BannerGrid,'Title','Current', ...
                'FontWeight','bold','BackgroundColor','white');
            app.CurrentPanel.Layout.Row = 1; app.CurrentPanel.Layout.Column = 2;
            gc1 = uigridlayout(app.CurrentPanel,[1 2]);
            gc1.ColumnWidth = {40,'1x'}; gc1.RowHeight={'1x'};
            gc1.Padding = [8 6 8 6]; gc1.ColumnSpacing = 8; gc1.BackgroundColor = 'white';
            app.CurrentLamp = uilamp(gc1,'Color',[0.65 0.65 0.65]);
            app.CurrentLamp.Layout.Row = 1; app.CurrentLamp.Layout.Column = 1;
            app.CurrentStatus = uilabel(gc1,'Text','Not run','FontWeight','bold','FontSize',14,'HorizontalAlignment','left');
            app.CurrentStatus.Layout.Row = 1; app.CurrentStatus.Layout.Column = 2;

            app.RecommendedPanel = uipanel(app.BannerGrid,'Title','Recommended', ...
                'FontWeight','bold','BackgroundColor','white');
            app.RecommendedPanel.Layout.Row = 1; app.RecommendedPanel.Layout.Column = 3;
            gc2 = uigridlayout(app.RecommendedPanel,[1 2]);
            gc2.ColumnWidth = {40,'1x'}; gc2.RowHeight={'1x'};
            gc2.Padding = [8 6 8 6]; gc2.ColumnSpacing = 8; gc2.BackgroundColor = 'white';
            app.RecommendedLamp = uilamp(gc2,'Color',[0.65 0.65 0.65]);
            app.RecommendedLamp.Layout.Row = 1; app.RecommendedLamp.Layout.Column = 1;
            app.RecommendedStatus = uilabel(gc2,'Text','Not run','FontWeight','bold','FontSize',14,'HorizontalAlignment','left');
            app.RecommendedStatus.Layout.Row = 1; app.RecommendedStatus.Layout.Column = 2;

            % ================= Content =================
            app.ContentGrid = uigridlayout(app.MainGrid,[2 2]);
            app.ContentGrid.Layout.Row = 2; app.ContentGrid.Layout.Column = 1;
            app.ContentGrid.ColumnWidth = {390, '1x'};
            app.ContentGrid.RowHeight = {'1x', 1};
            app.ContentGrid.Padding = [0 0 0 0];
            app.ContentGrid.ColumnSpacing = 10; app.ContentGrid.RowSpacing = 0;
            app.ContentGrid.BackgroundColor = bg;

            % left setup
            app.SetupPanel = uipanel(app.ContentGrid,'Title','Study setup','FontWeight','bold','BackgroundColor','white');
            app.SetupPanel.Layout.Row = 1; app.SetupPanel.Layout.Column = 1;
            app.SetupGrid = uigridlayout(app.SetupPanel,[2 1]);
            app.SetupGrid.RowHeight = {'1x', 52};
            app.SetupGrid.Padding = [8 8 8 8];
            app.SetupGrid.RowSpacing = 8;

            app.SetupTabs = uitabgroup(app.SetupGrid);
            app.SetupTabs.Layout.Row = 1; app.SetupTabs.Layout.Column = 1;
            app.BasicTab = uitab(app.SetupTabs,'Title','Basic');
            app.AdvancedTab = uitab(app.SetupTabs,'Title','Advanced');

            app.createBasicTab();
            app.createAdvancedTab();

            app.ButtonGrid = uigridlayout(app.SetupGrid,[1 4]);
            app.ButtonGrid.Layout.Row = 2; app.ButtonGrid.Layout.Column = 1;
            app.ButtonGrid.ColumnWidth = {'1x','1x','1x','1x'};
            app.ButtonGrid.Padding = [0 0 0 0];
            app.ButtonGrid.ColumnSpacing = 8;

            app.PreviewBtn = uibutton(app.ButtonGrid,'push','Text','Preview', ...
                'ButtonPushedFcn',@(~,~)app.previewCurrentCase());
            app.RunBtn = uibutton(app.ButtonGrid,'push','Text','Run', ...
                'ButtonPushedFcn',@(~,~)app.runAnalysis(),'FontWeight','bold');
            app.ExportBtn = uibutton(app.ButtonGrid,'push','Text','Export', ...
                'ButtonPushedFcn',@(~,~)app.exportDashboard());
            app.ResetBtn = uibutton(app.ButtonGrid,'push','Text','Reset', ...
                'ButtonPushedFcn',@(~,~)app.resetDefaults());

            app.RunBtn.BackgroundColor = navy;
            app.RunBtn.FontColor = [1 1 1];
            app.PreviewBtn.BackgroundColor = [0.93 0.95 0.98];
            app.ExportBtn.BackgroundColor = [0.93 0.95 0.98];
            app.ResetBtn.BackgroundColor = [0.96 0.96 0.96];

            % right workspace
            app.WorkspaceTabs = uitabgroup(app.ContentGrid);
            app.WorkspaceTabs.Layout.Row = 1; app.WorkspaceTabs.Layout.Column = 2;
            app.DashboardTab = uitab(app.WorkspaceTabs,'Title','Dashboard');
            app.ResultsTab = uitab(app.WorkspaceTabs,'Title','Bus results');

            app.DashboardGrid = uigridlayout(app.DashboardTab,[2 1]);
            app.DashboardGrid.RowHeight = {112,'1x'};
            app.DashboardGrid.Padding = [10 10 10 10];
            app.DashboardGrid.RowSpacing = 10;
            app.SummaryPanel = uipanel(app.DashboardGrid,'Title','Analysis summary','FontWeight','bold','BackgroundColor','white');
            app.SummaryPanel.Layout.Row = 1;
            sg = uigridlayout(app.SummaryPanel,[1 1]); sg.Padding = [6 6 6 6];
            app.SummaryText = uitextarea(sg,'Editable','off','FontName','Arial','FontSize',11.5);
            app.SummaryText.Value = {'Run analysis to populate the summary.'};

            app.PlotGrid = uigridlayout(app.DashboardGrid,[2 2]);
            app.PlotGrid.Layout.Row = 2;
            app.PlotGrid.RowHeight = {'1x','1x'};
            app.PlotGrid.ColumnWidth = {'1x','1x'};
            app.PlotGrid.Padding = [0 0 0 0];
            app.PlotGrid.RowSpacing = 10; app.PlotGrid.ColumnSpacing = 10;

            app.AxesA = uiaxes(app.PlotGrid); app.AxesA.Layout.Row = 1; app.AxesA.Layout.Column = 1;
            app.AxesB = uiaxes(app.PlotGrid); app.AxesB.Layout.Row = 1; app.AxesB.Layout.Column = 2;
            app.AxesC = uiaxes(app.PlotGrid); app.AxesC.Layout.Row = 2; app.AxesC.Layout.Column = 1;
            app.AxesD = uiaxes(app.PlotGrid); app.AxesD.Layout.Row = 2; app.AxesD.Layout.Column = 2;

            for ax = [app.AxesA, app.AxesB, app.AxesC, app.AxesD]
                ax.FontName = 'Arial';
                ax.FontSize = 10;
                ax.Toolbar.Visible = 'off';
                grid(ax,'on'); box(ax,'on');
            end
            title(app.AxesA,'Local passivation gains');
            title(app.AxesB,'Passivity index at limiting bus');
            title(app.AxesC,'Rightmost closed-loop eigenvalues');
            title(app.AxesD,'Disturbance response');

            app.ResultsGrid = uigridlayout(app.ResultsTab,[2 1]);
            app.ResultsGrid.RowHeight = {28,'1x'};
            app.ResultsGrid.Padding = [10 10 10 10];
            app.ResultsGrid.RowSpacing = 8;
            app.ResultsSummaryLabel = uilabel(app.ResultsGrid,'Text','Run analysis to populate the bus-wise table.','FontWeight','bold');
            app.ResultsTable = uitable(app.ResultsGrid,'Data',cell(0,0),'FontName','Arial');
            app.ResultsTable.RowStriping = 'on';
            app.ResultsTable.ColumnSortable = true;
            app.ResultsTable.Layout.Row = 2;

            app.setIdleStatus();
            app.updateResponsiveLayout();
        end

        function createBasicTab(app)
            app.BasicScroll = uipanel(app.BasicTab,'Scrollable','on','BorderType','none','BackgroundColor','white');
            app.BasicScroll.Position = [1 1 10 10];
            g = uigridlayout(app.BasicScroll,[3 1]);
            g.RowHeight = {185,170,160};
            g.Padding = [8 8 8 8];
            g.RowSpacing = 10;

            % 1 import
            p1 = uipanel(g,'Title','1. Grid data import','FontWeight','bold','BackgroundColor','white');
            d = uigridlayout(p1,[4 2]);
            d.RowHeight = {28,28,34,'1x'}; d.ColumnWidth = {95,'1x'};
            d.Padding = [8 8 8 8]; d.RowSpacing = 6; d.ColumnSpacing = 8;
            uilabel(d,'Text','Source');
            app.SourceType = uidropdown(d,'Items',{'Built-in MATPOWER case','Local MATPOWER file'}, ...
                'Value','Built-in MATPOWER case','ValueChangedFcn',@(~,~)app.previewCurrentCase());
            uilabel(d,'Text','Case / File');
            app.CaseField = uieditfield(d,'text','Value','case39','ValueChangedFcn',@(~,~)app.previewCurrentCase());
            app.BrowseBtn = uibutton(d,'push','Text','Browse local file ...','ButtonPushedFcn',@(~,~)app.browseCaseFile());
            app.BrowseBtn.Layout.Row = 3; app.BrowseBtn.Layout.Column = [1 2];
            app.CasePreviewText = uitextarea(d,'Editable','off','FontName','Arial','FontSize',10);
            app.CasePreviewText.Layout.Row = 4; app.CasePreviewText.Layout.Column = [1 2];

            % 2 placement
            p2 = uipanel(g,'Title','2. Converter placement','FontWeight','bold','BackgroundColor','white');
            p = uigridlayout(p2,[3 2]);
            p.RowHeight = {28, '1x', 28}; p.ColumnWidth = {110,'1x'};
            p.Padding = [8 8 8 8]; p.RowSpacing = 6; p.ColumnSpacing = 8;
            uilabel(p,'Text','Placement mode');
            app.PlacementMode = uidropdown(p,'Items',{'All buses (no-grounding required)'}, ...
                'Value','All buses (no-grounding required)','ValueChangedFcn',@(~,~)app.placementChanged());
            uilabel(p,'Text','Support buses');
            app.SupportBusField = uieditfield(p,'text','Placeholder','Disabled in no-grounding release','Enable','off');
            app.SupportBusField.Layout.Row = 2; app.SupportBusField.Layout.Column = 2;
            uilabel(p,'Text','Display bus (0 = auto)');
            app.DisplayBusField = uieditfield(p,'numeric','Value',0,'Limits',[0 Inf],'RoundFractionalValues','on');

            % 3 network
            p3 = uipanel(g,'Title','3. Network settings','FontWeight','bold','BackgroundColor','white');
            n = uigridlayout(p3,[4 2]);
            n.RowHeight = repmat({28},1,4); n.ColumnWidth = {120,'1x'};
            n.Padding = [8 8 8 8]; n.RowSpacing = 6; n.ColumnSpacing = 8;
            uilabel(n,'Text','Reactance scale'); app.XScale = uieditfield(n,'numeric','Value',1.5);
            uilabel(n,'Text','Resistance scale'); app.RScale = uieditfield(n,'numeric','Value',0.7);
            uilabel(n,'Text','Load scale'); app.LoadScale = uieditfield(n,'numeric','Value',1.0);
            uilabel(n,'Text','Grounding floor'); app.GSFloor = uieditfield(n,'numeric','Value',0,'LowerLimit',0,'Enable','off');
        end

        function createAdvancedTab(app)
            app.AdvancedScroll = uipanel(app.AdvancedTab,'Scrollable','on','BorderType','none','BackgroundColor','white');
            app.AdvancedScroll.Position = [1 1 10 10];
            g = uigridlayout(app.AdvancedScroll,[4 1]);
            g.RowHeight = {145,220,205,95};
            g.Padding = [8 8 8 8];
            g.RowSpacing = 10;

            % 4 converter + disturbance
            p1 = uipanel(g,'Title','4. Converter and disturbance settings','FontWeight','bold','BackgroundColor','white');
            c = uigridlayout(p1,[10 2]);
            c.RowHeight = repmat({28},1,10); c.ColumnWidth = {170,'1x'};
            c.Padding = [8 8 8 8]; c.RowSpacing = 6; c.ColumnSpacing = 8;
            uilabel(c,'Text','Observed voltage axis'); app.DistAxis = uidropdown(c,'Items',{'q','d'},'Value','q');
            uilabel(c,'Text','Disturbance amplitude'); app.IdAmp = uieditfield(c,'numeric','Value',1e-3);
            uilabel(c,'Text','Disturbance frequency (Hz)'); app.IdFreq = uieditfield(c,'numeric','Value',3.0);
            uilabel(c,'Text','Burst duration T_b (s)'); app.BurstDuration = uieditfield(c,'numeric','Value',0.8);
            uilabel(c,'Text','Filter reactance Xf'); app.Xf = uieditfield(c,'numeric','Value',0.15);
            uilabel(c,'Text','Filter resistance rf'); app.Rf = uieditfield(c,'numeric','Value',0.01);
            uilabel(c,'Text','Current-loop BW (Hz)'); app.WciHz = uieditfield(c,'numeric','Value',300);
            uilabel(c,'Text','Current-loop damping'); app.ZetaI = uieditfield(c,'numeric','Value',1.0);
            uilabel(c,'Text','Droop dp'); app.Dp = uieditfield(c,'numeric','Value',5.0);
            uilabel(c,'Text','Droop dq'); app.Dq = uieditfield(c,'numeric','Value',0.01);

            % 5 outer loop / support profile
            p2 = uipanel(g,'Title','5. Voltage loop and support-bus profile','FontWeight','bold','BackgroundColor','white');
            t = uigridlayout(p2,[6 2]);
            t.RowHeight = repmat({28},1,6); t.ColumnWidth = {180,'1x'};
            t.Padding = [8 8 8 8]; t.RowSpacing = 6; t.ColumnSpacing = 8;
            uilabel(t,'Text','Tau p / Tau q (s)');
            tauRow = uigridlayout(t,[1 2]); tauRow.Padding=[0 0 0 0]; tauRow.ColumnWidth={'1x','1x'}; tauRow.ColumnSpacing=8;
            app.TauP = uieditfield(tauRow,'numeric','Value',0.05);
            app.TauQ = uieditfield(tauRow,'numeric','Value',0.05);
            tauRow.Layout.Row = 1; tauRow.Layout.Column = 2;

            app.AutoTune = uicheckbox(t,'Text','Auto-tune Kpv / Kiv','Value',true,'ValueChangedFcn',@(~,~)app.toggleAutoTune());
            app.AutoTune.Layout.Row = 2; app.AutoTune.Layout.Column = [1 2];
            uilabel(t,'Text','Manual Kpv / Kiv');
            kvRow = uigridlayout(t,[1 2]); kvRow.Padding=[0 0 0 0]; kvRow.ColumnWidth={'1x','1x'}; kvRow.ColumnSpacing=8;
            app.Kpv = uieditfield(kvRow,'numeric','Value',1.5,'Enable','off');
            app.Kiv = uieditfield(kvRow,'numeric','Value',0.2,'Enable','off');
            kvRow.Layout.Row = 3; kvRow.Layout.Column = 2;

            app.SupportProfile = uicheckbox(t,'Text','Use milder support-bus profile','Value',true);
            app.SupportProfile.Layout.Row = 4; app.SupportProfile.Layout.Column = [1 2];
            uilabel(t,'Text','Support dp / dq');
            sup1 = uigridlayout(t,[1 2]); sup1.Padding=[0 0 0 0]; sup1.ColumnWidth={'1x','1x'}; sup1.ColumnSpacing=8;
            app.SupportDp = uieditfield(sup1,'numeric','Value',0.25);
            app.SupportDq = uieditfield(sup1,'numeric','Value',0.05);
            sup1.Layout.Row = 5; sup1.Layout.Column = 2;
            uilabel(t,'Text','Support Tau p / Tau q');
            sup2 = uigridlayout(t,[1 2]); sup2.Padding=[0 0 0 0]; sup2.ColumnWidth={'1x','1x'}; sup2.ColumnSpacing=8;
            app.SupportTauP = uieditfield(sup2,'numeric','Value',0.20);
            app.SupportTauQ = uieditfield(sup2,'numeric','Value',0.20);
            sup2.Layout.Row = 6; sup2.Layout.Column = 2;

            % 6 margins
            p3 = uipanel(g,'Title','6. Compensation margins','FontWeight','bold','BackgroundColor','white');
            m = uigridlayout(p3,[5 2]);
            m.RowHeight = repmat({28},1,5); m.ColumnWidth={170,'1x'};
            m.Padding=[8 8 8 8]; m.RowSpacing=6; m.ColumnSpacing=8;
            uilabel(m,'Text','Existing uniform g'); app.ExistingG = uieditfield(m,'numeric','Value',0,'LowerLimit',0);
            uilabel(m,'Text','Relative margin'); app.GMarginRel = uieditfield(m,'numeric','Value',0.02,'LowerLimit',0);
            uilabel(m,'Text','Absolute margin'); app.GMarginAbs = uieditfield(m,'numeric','Value',1e-4,'LowerLimit',0);
            uilabel(m,'Text','Eta safety margin'); app.EtaMargin = uieditfield(m,'numeric','Value',1e-7,'LowerLimit',0);
            uilabel(m,'Text','Stability margin target'); app.StabMargin = uieditfield(m,'numeric','Value',1e-5,'LowerLimit',0);

            % 7 usage
            p4 = uipanel(g,'Title','Usage note','FontWeight','bold','BackgroundColor','white');
            ig = uigridlayout(p4,[1 1]); ig.Padding=[6 6 6 6];
            app.AdvText = uitextarea(ig,'Editable','off','FontName','Arial','FontSize',10);
            app.AdvText.Value = {'Quick use:', ...
                '1) Keep case39 and All buses.', ...
                '2) Leave Display bus = 0 for automatic selection.', ...
                '3) Run analysis to compare current and recommended designs.', ...
                '4) No-grounding mode removes static Y_L, bus GS/BS, and GS floor.', ...
                '5) Eigenvalues are used for validation, not for local gain computation.'};
        end


        function updateResponsiveLayout(app)
            W = app.UIFigure.Position(3);
            app.BasicScroll.Position = [1 1 max(10, app.BasicTab.Position(3)-2) max(10, app.BasicTab.Position(4)-2)];
            app.AdvancedScroll.Position = [1 1 max(10, app.AdvancedTab.Position(3)-2) max(10, app.AdvancedTab.Position(4)-2)];

            if W >= 1450
                app.MainGrid.RowHeight = {120,'1x'};
                app.ContentGrid.ColumnWidth = {390,'1x'};
                app.ContentGrid.RowHeight = {'1x',1};
                app.SetupPanel.Layout.Row = 1; app.SetupPanel.Layout.Column = 1;
                app.WorkspaceTabs.Layout.Row = 1; app.WorkspaceTabs.Layout.Column = 2;
                app.BannerGrid.ColumnWidth = {'1x',135,135};
                app.BrandGrid.ColumnWidth = {260,'1x'};
                app.TitleLabel.FontSize = 26;
                app.CurrentStatus.FontSize = 14;
                app.RecommendedStatus.FontSize = 14;
                app.PlotGrid.RowHeight = {'1x','1x'};
                app.PlotGrid.ColumnWidth = {'1x','1x'};
                app.AxesA.Layout.Row = 1; app.AxesA.Layout.Column = 1;
                app.AxesB.Layout.Row = 1; app.AxesB.Layout.Column = 2;
                app.AxesC.Layout.Row = 2; app.AxesC.Layout.Column = 1;
                app.AxesD.Layout.Row = 2; app.AxesD.Layout.Column = 2;
            elseif W >= 1180
                app.MainGrid.RowHeight = {128,'1x'};
                app.ContentGrid.ColumnWidth = {'1x',1};
                app.ContentGrid.RowHeight = {470,'1x'};
                app.SetupPanel.Layout.Row = 1; app.SetupPanel.Layout.Column = 1;
                app.WorkspaceTabs.Layout.Row = 2; app.WorkspaceTabs.Layout.Column = 1;
                app.BannerGrid.ColumnWidth = {'1x',130,130};
                app.BrandGrid.ColumnWidth = {230,'1x'};
                app.TitleLabel.FontSize = 24;
                app.CurrentStatus.FontSize = 13;
                app.RecommendedStatus.FontSize = 13;
                app.PlotGrid.RowHeight = {'1x','1x'};
                app.PlotGrid.ColumnWidth = {'1x','1x'};
                app.AxesA.Layout.Row = 1; app.AxesA.Layout.Column = 1;
                app.AxesB.Layout.Row = 1; app.AxesB.Layout.Column = 2;
                app.AxesC.Layout.Row = 2; app.AxesC.Layout.Column = 1;
                app.AxesD.Layout.Row = 2; app.AxesD.Layout.Column = 2;
            else
                app.MainGrid.RowHeight = {145,'1x'};
                app.ContentGrid.ColumnWidth = {'1x',1};
                app.ContentGrid.RowHeight = {520,'1x'};
                app.SetupPanel.Layout.Row = 1; app.SetupPanel.Layout.Column = 1;
                app.WorkspaceTabs.Layout.Row = 2; app.WorkspaceTabs.Layout.Column = 1;
                app.BannerGrid.ColumnWidth = {'1x',120,120};
                app.BrandGrid.ColumnWidth = {190,'1x'};
                app.TitleLabel.FontSize = 22;
                app.CurrentStatus.FontSize = 12;
                app.RecommendedStatus.FontSize = 12;
                app.PlotGrid.RowHeight = {'1x','1x','1x','1x'};
                app.PlotGrid.ColumnWidth = {'1x'};
                app.AxesA.Layout.Row = 1; app.AxesA.Layout.Column = 1;
                app.AxesB.Layout.Row = 2; app.AxesB.Layout.Column = 1;
                app.AxesC.Layout.Row = 3; app.AxesC.Layout.Column = 1;
                app.AxesD.Layout.Row = 4; app.AxesD.Layout.Column = 1;
            end
        end

        function browseCaseFile(app)
            [f,p] = uigetfile({'*.m;*.mat','MATPOWER case files (*.m, *.mat)'}, 'Select a MATPOWER case file');
            if isequal(f,0), return; end
            app.SourceType.Value = 'Local MATPOWER file';
            app.CaseField.Value = fullfile(p,f);
            app.previewCurrentCase();
        end

        function previewCurrentCase(app)
            try
                app.setBusy(true, 'Loading case preview ...');
                cs = app.collectCaseSource();
                define_constants;
                mpc = loadcase(cs.loader);
                app.PreviewCase = mpc;
                app.CasePreviewText.Value = {
                    sprintf('Case: %s', cs.displayName)
                    sprintf('baseMVA: %.3g', mpc.baseMVA)
                    sprintf('Buses: %d', size(mpc.bus,1))
                    sprintf('Generators: %d', size(mpc.gen,1))
                    sprintf('Branches: %d', size(mpc.branch,1))
                    };
                app.setBusy(false, 'Case preview loaded.');
            catch ME
                app.CasePreviewText.Value = {['Error: ', ME.message]};
                app.setBusy(false, 'Case preview failed.');
            end
        end

        function cs = collectCaseSource(app)
            val = strtrim(string(app.CaseField.Value));
            if strcmp(app.SourceType.Value, 'Local MATPOWER file')
                if strlength(val)==0 || exist(val,'file')~=2
                    error('Please choose a valid local MATPOWER case file.');
                end
                [~,nm,ext] = fileparts(char(val));
                cs = struct('type','file','file',char(val),'name','','loader',char(val),'displayName',[nm ext]);
            else
                if strlength(val)==0
                    val = "case39";
                    app.CaseField.Value = char(val);
                end
                cs = struct('type','builtin','file','','name',char(val),'loader',char(val),'displayName',char(val));
            end
        end

        function placementChanged(app)
            isAll = strcmp(app.PlacementMode.Value,'All buses (no-grounding required)');
            app.SupportBusField.Enable = ternaryLocal(~isAll,'on','off');
            if isAll
                app.SupportBusField.Value = '';
            end
        end

        function toggleAutoTune(app)
            en = ternaryLocal(app.AutoTune.Value, 'off', 'on');
            app.Kpv.Enable = en;
            app.Kiv.Enable = en;
        end

        function resetDefaults(app)
            app.SourceType.Value = 'Built-in MATPOWER case';
            app.CaseField.Value = 'case39';
            app.PlacementMode.Value = 'All buses (no-grounding required)';
            app.SupportBusField.Value = '';
            app.DisplayBusField.Value = 0;
            app.XScale.Value = 1.5;
            app.RScale.Value = 0.7;
            app.LoadScale.Value = 1.0;
            app.GSFloor.Value = 0;
            app.DistAxis.Value = 'q';
            app.IdAmp.Value = 1e-3;
            app.IdFreq.Value = 3.0;
            app.BurstDuration.Value = 0.8;
            app.Xf.Value = 0.15;
            app.Rf.Value = 0.01;
            app.WciHz.Value = 300;
            app.ZetaI.Value = 1.0;
            app.Dp.Value = 5.0;
            app.Dq.Value = 0.01;
            app.TauP.Value = 0.05;
            app.TauQ.Value = 0.05;
            app.AutoTune.Value = true;
            app.toggleAutoTune();
            app.Kpv.Value = 1.5;
            app.Kiv.Value = 0.2;
            app.SupportProfile.Value = true;
            app.SupportDp.Value = 0.25;
            app.SupportDq.Value = 0.05;
            app.SupportTauP.Value = 0.20;
            app.SupportTauQ.Value = 0.20;
            app.ExistingG.Value = 0;
            app.GMarginRel.Value = 0.02;
            app.GMarginAbs.Value = 1e-4;
            app.EtaMargin.Value = 1e-7;
            app.StabMargin.Value = 1e-5;
            app.setIdleStatus();
        end

        function setIdleStatus(app)
            grey = [0.67 0.67 0.67];
            app.CurrentLamp.Color = grey;
            app.RecommendedLamp.Color = grey;
            app.CurrentStatus.Text = 'Not run';
            app.RecommendedStatus.Text = 'Not run';
            app.SummaryText.Value = {'Run analysis to populate the summary.'};
            app.ResultsSummaryLabel.Text = 'Run analysis to populate the bus-wise table.';
            app.ResultsTable.Data = cell(0,0);
            for ax = [app.AxesA, app.AxesB, app.AxesC, app.AxesD]
                cla(ax); title(ax,''); xlabel(ax,''); ylabel(ax,'');
            end
            title(app.AxesA,'Local passivation gains');
            title(app.AxesB,'Passivity index at limiting bus');
            title(app.AxesC,'Rightmost closed-loop eigenvalues');
            title(app.AxesD,'Disturbance response');
        end

        function user = collectInputs(app)
            cs = app.collectCaseSource();
            user.caseSource = struct('type', cs.type, 'name', cs.name, 'file', cs.file);
            user.f0 = 60;
            user.useAllBuses = strcmp(app.PlacementMode.Value,'All buses (no-grounding required)');
            if user.useAllBuses
                user.supportBusIDs = [];
            else
                txt = strtrim(app.SupportBusField.Value);
                if isempty(txt)
                    user.supportBusIDs = [];
                else
                    toks = regexp(txt, '[,;\s]+', 'split');
                    nums = str2double(toks);
                    user.supportBusIDs = nums(~isnan(nums));
                end
            end
            user.displayBus = app.DisplayBusField.Value;
            user.existingGUniform = app.ExistingG.Value;
            user.stress = struct('x_scale', app.XScale.Value, 'r_scale', app.RScale.Value, 'load_scale', app.LoadScale.Value);
            user.paper_match = struct('remove_taps_and_shifts', true, 'remove_bus_shunts', true, 'gs_floor_pu', app.GSFloor.Value);
            user.netopt = struct('use_static_load_admittance', false, 'c_min', 1e-6, 'g_eps', 0);

            prm = struct();
            prm.Xf = app.Xf.Value;
            prm.rf = app.Rf.Value;
            prm.w_ci_Hz = app.WciHz.Value;
            prm.zeta_i = app.ZetaI.Value;
            prm.dp = app.Dp.Value;
            prm.dq = app.Dq.Value;
            prm.tau_p = app.TauP.Value;
            prm.tau_q = app.TauQ.Value;
            prm.Kpv = app.Kpv.Value;
            prm.Kiv = app.Kiv.Value;
            user.prm = prm;
            user.useAutoTuneKV = app.AutoTune.Value;
            user.support_profile = struct('enable', app.SupportProfile.Value, ...
                'dp_non_gen', app.SupportDp.Value, 'dq_non_gen', app.SupportDq.Value, ...
                'tau_p_non_gen', app.SupportTauP.Value, 'tau_q_non_gen', app.SupportTauQ.Value);
            user.g_margin_rel = app.GMarginRel.Value;
            user.g_margin_abs = app.GMarginAbs.Value;
            user.eta_margin_abs = app.EtaMargin.Value;
            user.stable_margin_target = app.StabMargin.Value;
            user.time_response = struct('axis', app.DistAxis.Value, 'id_amp', app.IdAmp.Value, ...
                'id_f_Hz', app.IdFreq.Value, 'burst_duration', app.BurstDuration.Value, ...
                't_end', 1.2, 'nPoints', 2400);
        end

        function runAnalysis(app)
            try
                app.setBusy(true, 'Running analysis ...');
                drawnow;
                out = hku_passivity_engine(app.collectInputs());
                app.LastResult = out;
                app.renderResults(out);
                app.setBusy(false, 'Analysis completed.');
            catch ME
                app.setBusy(false, 'Analysis failed.');
                uialert(app.UIFigure, ME.message, 'Analysis failed', 'Icon','error');
            end
        end

        function renderResults(app, out)
            % status cards
            app.CurrentLamp.Color = ternaryColor(out.summary.currentStable, [0.16 0.66 0.27], [0.85 0.16 0.16]);
            app.RecommendedLamp.Color = ternaryColor(out.summary.recommendedStable, [0.16 0.66 0.27], [0.85 0.16 0.16]);
            app.CurrentStatus.Text = ternaryLocal(out.summary.currentStable, 'Stable', 'Unstable');
            app.RecommendedStatus.Text = ternaryLocal(out.summary.recommendedStable, 'Stable', 'Unstable');

            % selected display bus for response; limiting bus for eta plot
            limitingBus = out.summary.maxGminBus;
            displayBus = out.summary.displayBus;
            sameBus = (limitingBus == displayBus);

            summaryLines = {
                sprintf('Case: %s | Converter buses: %d | Network model: no-grounding current-balance', ...
                    out.summary.caseName, out.summary.numConverters)
                sprintf('Current design: %s, max Re(lambda)=%.4e | Recommended design: %s, max Re(lambda)=%.4e', ...
                    ternaryLocal(out.summary.currentStable,'stable','unstable'), out.summary.maxReCurrent, ...
                    ternaryLocal(out.summary.recommendedStable,'stable','unstable'), out.summary.maxReRecommended)
                sprintf('Limiting bus: %d | g_min=%.4g | largest additional g=%.4g at bus %d | f*=%.4g Hz', ...
                    out.summary.maxGminBus, out.summary.maxGmin, out.summary.maxAdditionalNeeded, ...
                    out.summary.maxAdditionalBus, out.summary.fStarDisplayHz)
                sprintf('Displayed response bus: %d | local gain computation is decentralized; eigenvalues are validation only.', ...
                    displayBus)
                };
            if isfield(out.summary, 'noGroundingNetwork') && out.summary.noGroundingNetwork
                summaryLines{end+1} = sprintf('Network diagnostic: common-mode pair %.3e%+.3ej / %.3e%+.3ej; remaining max Re=%.3e.', ...
                    real(out.summary.netLambdaPlus), imag(out.summary.netLambdaPlus), ...
                    real(out.summary.netLambdaMinus), imag(out.summary.netLambdaMinus), ...
                    out.summary.netMaxReExcludingCommon);
            end
            if ~sameBus
                summaryLines{end+1} = sprintf('Note: Display bus differs from limiting bus; passivity-index plot still uses the limiting bus %d.', limitingBus);
            end
            app.SummaryText.Value = summaryLines;

            app.ResultsSummaryLabel.Text = sprintf('Current design: %s   |   Recommended design: %s   |   Limiting bus: %d   |   Display bus: %d', ...
                ternaryLocal(out.summary.currentStable,'Stable','Unstable'), ...
                ternaryLocal(out.summary.recommendedStable,'Stable','Unstable'), ...
                limitingBus, displayBus);
            app.ResultsTable.Data = out.table;

            % Plot A - gains
            ax = app.AxesA; cla(ax); hold(ax,'on');
            busID = out.table.bus_id;
            bar(ax, busID, out.rec.g, 0.70, 'FaceColor',[0.86 0.90 0.96], 'EdgeColor',[0.35 0.40 0.48], 'LineWidth',0.8);
            plot(ax, busID, out.gains.gmin, 'o-', 'Color',[0.08 0.18 0.36], 'MarkerFaceColor','w', 'MarkerSize',4.5, 'LineWidth',1.15);
            plot(ax, busID, out.gains.g0_LF, 's--', 'Color',[0.42 0.45 0.50], 'MarkerFaceColor','w', 'MarkerSize',4.0, 'LineWidth',1.0);
            xlabel(ax,'Bus ID'); ylabel(ax,'g (p.u.)'); title(ax,'Local passivation gains');
            legend(ax, {'Recommended g','Computed g_{min}','Low-frequency g_0'}, 'Location','northwest');
            idxLim = find(busID == limitingBus, 1);
            if ~isempty(idxLim)
                plot(ax, limitingBus, out.gains.gmin(idxLim), 'p', 'Color',[0.08 0.18 0.36], 'MarkerSize',8, 'MarkerFaceColor',[0.08 0.18 0.36], 'HandleVisibility','off');
                text(ax, limitingBus+0.2, out.gains.gmin(idxLim)+0.02*max(out.rec.g), sprintf('bus %d', limitingBus), 'FontName','Arial', 'FontSize',10);
            end
            grid(ax,'on'); box(ax,'on');

            % Plot B - passivity index at LIMITING bus
            ax = app.AxesB; cla(ax); hold(ax,'on');
            idxLimOp = find([out.op.bus_id] == limitingBus, 1);
            [lam_w_lim, fHz_lim] = lambda_max_sym_vs_w_local(out.conv(idxLimOp).sysY_dev, out.cfg.w_grid);
            eta_cur_lim = out.cur.g(idxLimOp) - lam_w_lim;
            eta_rec_lim = out.rec.g(idxLimOp) - lam_w_lim;
            [etaMinLim, idxMinLim] = min(eta_cur_lim); %#ok<ASGLU>
            fStarLim = fHz_lim(idxMinLim);
            semilogx(ax, fHz_lim, eta_cur_lim, '-', 'Color',[0.10 0.10 0.10], 'LineWidth',1.15);
            semilogx(ax, fHz_lim, eta_rec_lim, '--', 'Color',[0.00 0.35 0.70], 'LineWidth',1.20);
            ax.XScale = 'log';
            yline(ax,0,'k:','LineWidth',0.9);
            xline(ax,fStarLim,'k:','LineWidth',0.9,'HandleVisibility','off');
            xlabel(ax,'Frequency (Hz)'); ylabel(ax,'eta_i(omega) (p.u.)');
            title(ax, sprintf('Passivity index at limiting bus %d', limitingBus));
            legend(ax, {'Current','Recommended'}, 'Location','northeast');
            text(ax, 0.08, 0.92, sprintf('bus %d', limitingBus), 'Units','normalized', 'FontSize',10, 'FontName','Arial');
            grid(ax,'on'); box(ax,'on');

            % Plot C - eigenvalues
            ax = app.AxesC; cla(ax); hold(ax,'on');
            [hCur, hRHP, hRec] = plot_rightmost_eigs_local(ax, out.cur.eig, out.rec.eig, 40);
            xlabel(ax,'Real part (1/s)'); ylabel(ax,'Imag part (rad/s)');
            title(ax,'Rightmost closed-loop eigenvalues');
            if isgraphics(hRHP)
                legend(ax, [hCur hRHP hRec], {'Current design','Current RHP modes','Recommended design'}, 'Location','northeast');
            else
                legend(ax, [hCur hRec], {'Current design','Recommended design'}, 'Location','northeast');
            end
            grid(ax,'on'); box(ax,'on');

            % Plot D - response at DISPLAY bus
            ax = app.AxesD; cla(ax); hold(ax,'on');
            plot(ax, out.time.t, out.time.y_current, 'k-', 'LineWidth',1.15);
            plot(ax, out.time.t, out.time.y_recommended, 'k--', 'LineWidth',1.15);
            xline(ax, out.time.info.burst_duration, 'k:', 'LineWidth',0.9, 'HandleVisibility','off');
            xlabel(ax,'Time (s)');
            ylabel(ax, sprintf('Delta v_%s,%d(t) (p.u.)', lower(out.time.info.axis), displayBus));
            title(ax, sprintf('Disturbance response at bus %d', displayBus));
            legend(ax, {'Current','Recommended'}, 'Location','southeast');
            grid(ax,'on'); box(ax,'on');
        end

        function exportDashboard(app)
            if isempty(app.LastResult)
                uialert(app.UIFigure,'Please run an analysis before exporting the dashboard.','Nothing to export');
                return
            end
            [f,p] = uiputfile({'*.png';'*.pdf';'*.fig'}, 'Export dashboard');
            if isequal(f,0), return; end
            fp = fullfile(p,f);
            ff = figure('Color','w','Renderer','painters','Position',[100 100 1280 920]);
            tl = tiledlayout(ff,2,2,'TileSpacing','compact','Padding','compact');
            ax1 = nexttile(tl,1); copyobj(allchild(app.AxesA), ax1); syncAxesStyle_local(app.AxesA, ax1);
            ax2 = nexttile(tl,2); copyobj(allchild(app.AxesB), ax2); syncAxesStyle_local(app.AxesB, ax2);
            ax3 = nexttile(tl,3); copyobj(allchild(app.AxesC), ax3); syncAxesStyle_local(app.AxesC, ax3);
            ax4 = nexttile(tl,4); copyobj(allchild(app.AxesD), ax4); syncAxesStyle_local(app.AxesD, ax4);
            [~,~,ext] = fileparts(fp);
            switch lower(ext)
                case '.pdf'
                    exportgraphics(ff, fp, 'ContentType','vector');
                case '.fig'
                    savefig(ff, fp);
                otherwise
                    exportgraphics(ff, fp, 'Resolution', 300);
            end
            close(ff);
        end

        function setBusy(app, tf, msg)
            app.UIFigure.Pointer = ternaryLocal(tf, 'watch', 'arrow');
            app.RunBtn.Enable = ternaryLocal(~tf, 'on', 'off');
            app.ExportBtn.Enable = ternaryLocal(~tf, 'on', 'off');
            app.UIFigure.Name = ['HKU ECE Passivity-Based Stability Workbench - ', msg];
            drawnow limitrate;
        end
    end
end

function out = ternaryLocal(cond, a, b)
if cond, out = a; else, out = b; end
end

function c = ternaryColor(cond, cTrue, cFalse)
if cond, c = cTrue; else, c = cFalse; end
end

function [hCur, hRHP, hRec] = plot_rightmost_eigs_local(ax, eigCur, eigRec, K)
re0 = real(eigCur); im0 = imag(eigCur);
re1 = real(eigRec); im1 = imag(eigRec);
K0 = min(K, numel(eigCur)); K1 = min(K, numel(eigRec));
[~, idx0] = maxk(re0, K0); [~, idx1] = maxk(re1, K1);
idx0 = idx0(:); idx1 = idx1(:);
isRHP = re0(idx0) > 1e-8;
hCur = plot(ax, re0(idx0(~isRHP)), im0(idx0(~isRHP)), 'ko', ...
    'MarkerSize',5.8, 'LineWidth',1.0, 'MarkerFaceColor','w'); hold(ax,'on');
if any(isRHP)
    hRHP = plot(ax, re0(idx0(isRHP)), im0(idx0(isRHP)), 'ro', ...
        'MarkerSize',6.6, 'LineWidth',1.25, 'MarkerFaceColor','w');
else
    hRHP = gobjects(1);
end
hRec = plot(ax, re1(idx1), im1(idx1), 'bx', 'MarkerSize',6.2, 'LineWidth',1.1);
xline(ax, 0, 'k:', 'LineWidth',0.9, 'HandleVisibility','off');
end

function [lambdaMax, fHz] = lambda_max_sym_vs_w_local(sysY, w_grid)
Y = freqresp(sysY, w_grid);
lambdaMax = zeros(numel(w_grid),1);
for ii = 1:numel(w_grid)
    Yjw = Y(:,:,ii);
    S = 0.5*(Yjw + Yjw');
    lambdaMax(ii) = max(real(eig(S)));
end
fHz = w_grid/(2*pi);
end

function syncAxesStyle_local(axSrc, axDst)
axDst.XScale = axSrc.XScale;
axDst.YScale = axSrc.YScale;
axDst.XLim = axSrc.XLim;
axDst.YLim = axSrc.YLim;
axDst.FontName = 'Arial';
axDst.FontSize = 10;
xlabel(axDst, axSrc.XLabel.String);
ylabel(axDst, axSrc.YLabel.String);
title(axDst, axSrc.Title.String);
grid(axDst,'on'); box(axDst,'on');
end
