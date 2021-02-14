function [eq] = Eq_maker(Element_Type, Element_no, total_no_elements, varargin)
%EQ_MAKER This function returns the equations of a corresponding element in
%the MSA Model
% Rigid link parameters are just 'd' vector
% Flexible link parameters are K11, K12, K21, K22
% Elastic joint parameters are lamdaR & lamdaE & Ka
% Passive joint parameters are lamdaR & lamdaP

    Total_w = total_no_elements + 1;
    Total_T = total_no_elements + 1;
    
    switch Element_Type
        case 'rigid_link' 
            d = skew(varargin{1});
            D = [eye(3), d'; zeros(3), eye(3)];
            eq = [ zeros(6, 6 * (Total_w)) zeros(6, 6 * (Element_no - 1)) D -eye(6,6) zeros(6, 6 * (Total_T - Element_no - 1));
                   zeros(6, 6 * (Element_no - 1)) eye(6,6) D' zeros(6, 6 * (Total_w - (Element_no+1))) zeros(6, 6*Total_T)];
        case 'flexible_link'
            K11 = varargin{1};
            K12 = varargin{2};
            K21 = varargin{3};
            K22 = varargin{4};
            eq = [ zeros(6, 6 * (Element_no - 1)) -eye(6,6) zeros(6, 6 * (Total_w - Element_no)) zeros(6, 6 * (Element_no - 1)) K11 K12 zeros(6, 6 * (Total_T - Element_no - 1));
                   zeros(6, 6 * (Element_no)) -eye(6,6) zeros(6, 6 * (Total_w - (Element_no+1))) zeros(6, 6 * (Element_no - 1)) K21 K22 zeros(6, 6 * (Total_T - Element_no - 1))];
        case 'elastic_joint'
            lamda_r = varargin{1};
            lamda_e = varargin{2};
            Ka = varargin{3};

            eq = [ zeros(5, 6 * (Total_w)) zeros(5, 6 * (Element_no - 1)) lamda_r -lamda_r zeros(5, 6 * (Total_T - Element_no - 1));
                   zeros(6, 6 * (Element_no - 1)) eye(6,6) eye(6,6) zeros(6, 6 * (Total_w - Element_no -1)) zeros(6, 6*Total_T);
                   zeros(1, 6 * (Element_no - 1)) lamda_e zeros(1,6 * (Total_w - Element_no)) zeros(1, 6 * (Element_no - 1))...
                   Ka*lamda_e -Ka*lamda_e zeros(1, 6 * (Total_T - Element_no - 1))];
        
        case 'rigid_joint'
            
        case 'passive_joint'
            lamda_r = varargin{1};
            lamda_p = varargin{2};
            
            eq = [ zeros(5, 6 * (Total_w)) zeros(5, 6 * (Element_no - 1)) lamda_r -lamda_r zeros(5, 6 * (Total_T - Element_no - 1));
                   zeros(5, 6 * (Element_no - 1)) lamda_r lamda_r zeros(5, 6 * (Total_w - Element_no -1)) zeros(5, 6*Total_T);
                   zeros(1, 6 * (Element_no - 1)) lamda_p zeros(1, 6 * (Total_w - Element_no)) zeros(1, 6*Total_T);
                   zeros(1, 6 * (Element_no)) lamda_p zeros(1, 6 * (Total_w - Element_no - 1)) zeros(1, 6*Total_T) ];
        case 'rigid_base'
            eq = [ zeros(6, 6 * Total_w) zeros(6, 6 * (Element_no - 1)) eye(6,6) zeros(6, 6 * (Total_T - Element_no)) ];
        case 'external_force'
            eq = [ zeros(6, 6 * (Element_no)) -eye(6,6) zeros(6, 6 * Total_T) ];
            
        otherwise
            warning('Unexpected element type')
    end

end

