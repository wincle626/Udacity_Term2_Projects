function ctrl = pidctrl(p, i, d, p_e, i_e, d_e)    
	ctrl = - p*p_e - i*i_e - d*d_e;
end