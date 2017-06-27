def parse_llh_msgs(data):
    """Returns a list contaning data in LLH format
        Args:
            data: string input in LLH format from RTKLib
        Returns:
            Dictionary containing GPS data
        """
    data_format = ['GPSW', 'TOW', 'lat', 'lon', 'alt', 'Q', 'ns', 'sdn', 'sde',
                   'sdu', 'sdne', 'sdeu', 'sdun', 'age', 'ratio']
    outp = {}

    all_vals = data.split()
    num_vals = len(all_vals)

    if num_vals == len(data_format):
        for i in range(num_vals):
            if i < 2:
                continue
            outp[data_format[i]] = float(all_vals[i])

        return outp
    else:
        return None
