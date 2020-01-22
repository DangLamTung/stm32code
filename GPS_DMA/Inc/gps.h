typedef struct
{
	uint8_t			UTC_Hour;
	uint8_t			UTC_Min;
	uint8_t			UTC_Sec;
	uint16_t		UTC_MicroSec;

	char		    Pos;
	uint8_t			Lat_Deg;
	float			Lat_Minute;

	char            Lat_Dir;
	uint8_t         Lon_Deg;
	float           Lon_Minute;
	char            Lon_Dir;
	float			Speed;
	double			Track;
	unsigned int    date;

	float			mag_v;
	char			var_dir;
	char			mode_ind;
}GNRMC;
