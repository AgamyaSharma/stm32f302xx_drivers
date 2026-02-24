void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI2){
			SPI2_PCLK_ENABLE();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_ENABLE();

		}
	}else{
		if(pSPIx == SPI2){
			SPI2_PCLK_DISABLE();
		}else if(pSPIx == SPI3){
			SPI2_PCLK_DISABLE();
	}
}





void SPI_Innit(SPI_Handle_t *pSPIHandle){

}
