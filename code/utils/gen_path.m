function os_path = gen_path(path_in)

  os_path = path_in{1};
  for p = path_in(2:end)
    os_path = [os_path filesep p{1}];
  end
  
end