function [discrete_state] = get_discrete_state(state, os_low, os_win_size)
    discrete_state = cast((state - os_low) ./ os_win_size, 'int16');
    discrete_state = discrete_state + 1;
    
    % ensure discrete state does not become larger than max
    discrete_state(discrete_state > 21) = 21;
    discrete_state(discrete_state < 1) = 1;
end