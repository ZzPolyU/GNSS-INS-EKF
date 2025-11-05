classdef GNSSData < handle
    properties
        gnss_data;
    end
    
    methods
        function obj = GNSSData(obs_file, nav_file)
            obj.gnss_data.nav = gt.Gnav(nav_file);
            obj.gnss_data.obs = gt.Gobs(obs_file);

            obj.gnss_data.obs.plot
        end

        function gnss_data = get_gnss_data(obj)
            gnss_data = obj.gnss_data;
        end
    end
end
