

#pragma pack(1)
struct TX_message_data
{
    float UAV_N;
    float UAV_E;
    float UAV_D;
    float WP_case;
};
#pragma pack()

#pragma pack(2)
struct RX_message_data
{
    float R_UAV_N;
    float R_UAV_E;
    float R_UAV_D;
    float R_WP_case;
};
#pragma pack()

#pragma pack()
struct struct_DATA
{
	// ----
	// Data
	// ----

	int16_t		sIDLMajorVersion;
	int16_t		sIDLMinorVersion;
	int16_t		sSource;
	float		fTime;

	char		cID[8];
	char		cType[8];
	int16_t		sACcat;

	char		cLivery[8];
	double		dLat;
	double		dLon;
	int64_t		lAltWGS84;
	int32_t		lAltMSL;
	int32_t		lAltAGL;
	float   	fBRNG;
	float   	fRNG;
	float   	fELV;

	float   	fTHDG;
	float   	fPitch;
	float   	fRoll;

	float   	fTTRK;
	float   	fVV;
	float   	fGS;
	float   	fTAS;
	float   	fCR;

	int16_t   	sGearPos;
	int16_t		sFlapPos;
	int16_t		sSquawk;
	int64_t		lModeSAddr;
	int16_t		sNACp;
	int16_t		sNIC;
	int16_t		sSIL;

	int16_t		sADSBstat;
	int16_t		sTCASstat;
	int16_t		sDISPmatrix;
	int16_t		sAGstatus;
	int16_t		sVertSense;
	int16_t		sQLbits;
	int16_t		sLWbits;

	float   	fBRNGacc;
	float   	fRNGacc;
	float   	fELVacc;
	float   	fHDGacc;
	float   	fCRacc;
	float   	fTCASvertrate;
	int16_t   	sTCASVadvisory;

	float   	fTCAShdgmd;
	int16_t   	sTCASHadvisory;

	int16_t   	sWP;
	float    	fWPlat[4];
	float	    	fWPlon[4];
	float	    	fWpalt[4];
	float	    	fWPspd[4];
	int16_t	    	sFlags1;
	int16_t		sFlags2;
	int16_t 	sFlags3;
};
#pragma pack()
