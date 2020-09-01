#ifndef MATRIX_INVERSE_CALCULATION

#else

		// Matrix initialisation
		srcRows = N_MIC;
		srcColumns = 1;
		arm_mat_init_f32(&matXf, srcRows, srcColumns, vect_Xf);


		srcRows = 1;
		srcColumns = N_MIC;
		arm_mat_init_f32(&matXfh, srcRows, srcColumns, vect_Xfh);

		uint16_t f = 0;


		float vect_Rf_real_inv[N_MIC*N_MIC];
		float vect_Rf_imag_inv[N_MIC*N_MIC];
		arm_matrix_instance_f32 mat_Rf_real_inv;
		arm_matrix_instance_f32 mat_Rf_imag_inv;

		float vect_Rf_real[N_MIC*N_MIC];
		float vect_Rf_imag[N_MIC*N_MIC];
		arm_matrix_instance_f32 mat_Rf_real;
		arm_matrix_instance_f32 mat_Rf_imag;

		float vect_interm_1[N_MIC*N_MIC];
		float vect_interm_2[N_MIC*N_MIC];
		arm_matrix_instance_f32 mat_interm_1;
		arm_matrix_instance_f32 mat_interm_2;

		// Frequency bin processing
		for (f = 0; f < FFTSIZE; f += 2) {


			/*
			 * Xf = [mic1_f_real, mic1_f_imag,
			 * 		 mic2_f_real, mic2_f_imag,
			 * 		 mic3_f_real, mic3_f_imag,
			 * 		 mic4_f_real, mic4_f_imag ]
			 */
			vect_Xf[0] = left_1_f[f];
			vect_Xf[1] = left_1_f[f + 1];
			vect_Xf[2] = left_3_f[f];
			vect_Xf[3] = left_3_f[f + 1];
			vect_Xf[4] = right_1_f[f];
			vect_Xf[5] = right_1_f[f + 1];
			vect_Xf[6] = right_3_f[f];
			vect_Xf[7] = right_3_f[f + 1];

			/*
			 * Xfh = Xfconj
			 */
			arm_cmplx_conj_f32(vect_Xf, vect_Xfh, N_MIC);


			srcRows = N_MIC;
			srcColumns = N_MIC;
			arm_mat_init_f32(&matR[f], srcRows, srcColumns, vect_R[f]);


			// TESTING FD
			/* square example

			float vectresult[8];

			//float vectX[4] = {1, -1, 1, 1};
			//float vectXh[4] = {1, 1, 1, -1};
			//works ok: 2 0 0 -2 0 2 2 0

			//float vectX[4] = {1, 0, -1, 0};
			//float vectXh[4] = {1, 0, 1, 0};
			//works ok: 1 0 1 0 -1 0 -1 0

			srcRows = 2;
			srcColumns = 1;
			arm_mat_init_f32(&matX, srcRows, srcColumns, vectX);
			srcRows = 1;
			srcColumns = 2;
			arm_mat_init_f32(&matXh, srcRows, srcColumns, vectXh);
			srcRows = 2;
			srcColumns = 2;
			arm_mat_init_f32(&result, srcRows, srcColumns, vectresult);
			status = arm_mat_cmplx_mult_f32(&matX, &matXh, &result);
			*/

			/* rectangular example
			float vectresult[12];
			float vectX[6] = {1, -1, 1, 1, 1, -1};
			float vectXh[4] = {1, 1, 1, -1};
			//works ok: 2 0 0 -2 0 2 2 0 2 0 0 -2

			srcRows = 3;
			srcColumns = 1;
			arm_mat_init_f32(&matX, srcRows, srcColumns, vectX);
			srcRows = 1;
			srcColumns = 2;
			arm_mat_init_f32(&matXh, srcRows, srcColumns, vectXh);
			srcRows = 3;
			srcColumns = 2;
			arm_mat_init_f32(&result, srcRows, srcColumns, vectresult);
			status = arm_mat_cmplx_mult_f32(&matX, &matXh, &result);
			*/
			// TESTING FD

			/*
			 * R = Xf*Xfconj
			 */
			status = arm_mat_cmplx_mult_f32(&matXf, &matXfh, &matR[f]);

#define LAMBDA 0.01

			/*
			 * Rf_real = real(R)
			 * Rf_imag = imag(R)
			 */
			for(uint8_t i = 0; i < N_MIC*N_MIC; i ++){
				vect_Rf_real[i] = vect_R[f][2*i];
				vect_Rf_imag[i] = vect_R[f][2*i+1];
			}

			for(uint8_t i = 0; i < N_MIC; i++){
				vect_Rf_real[i*N_MIC+i] += LAMBDA;
				vect_Rf_imag[i*N_MIC+i] += LAMBDA;
			}

			arm_mat_init_f32(&mat_Rf_real, N_MIC, N_MIC, vect_Rf_real);
			arm_mat_init_f32(&mat_Rf_imag, N_MIC, N_MIC, vect_Rf_imag);
			arm_mat_init_f32(&mat_Rf_real_inv, N_MIC, N_MIC, vect_Rf_real_inv);
			arm_mat_init_f32(&mat_Rf_imag_inv, N_MIC, N_MIC, vect_Rf_imag_inv);
			arm_mat_init_f32(&mat_interm_1, N_MIC, N_MIC, vect_interm_1);
			arm_mat_init_f32(&mat_interm_2, N_MIC, N_MIC, vect_interm_2);

			//Compute A^-1 and B^-1
			arm_mat_inverse_f32(&mat_Rf_real, &mat_Rf_real_inv);
			arm_mat_inverse_f32(&mat_Rf_imag, &mat_Rf_imag_inv);

			/*
			 * Revive data
			 * Rf_real = real(R)
			 * Rf_imag = imag(R)
			 */
			for(uint8_t i = 0; i < N_MIC*N_MIC; i ++){
				vect_Rf_real[i] = vect_R[f][2*i];
				vect_Rf_imag[i] = vect_R[f][2*i+1];
			}

			for(uint8_t i = 0; i < N_MIC; i++){
				vect_Rf_real[i*N_MIC+i] += LAMBDA;
				vect_Rf_imag[i*N_MIC+i] += LAMBDA;
			}

			/*
			 * Rinv = R^-1 = (A + B*A^-1*B)^-1 - i*(B + A*B^-1*A)^-1
			 */

			srcRows = N_MIC;
			srcColumns = N_MIC;
			arm_mat_init_f32(&matRinv[f], srcRows, srcColumns, vect_Rinv[f]);

			// interm_1 = B*A^-1
			arm_mat_mult_f32(&mat_Rf_imag, &mat_Rf_real_inv, &mat_interm_1);

			// interm_2 = interm_1*B = B*A^-1*B
			arm_mat_mult_f32(&mat_interm_1, &mat_Rf_imag, &mat_interm_2);

			// interm_1 = A+interm_2 = A + B*A^-1*B
			arm_mat_add_f32(&mat_Rf_real, &mat_interm_2, &mat_interm_1);

			// interm_2 = interm_1^-1 =(A + B*A^-1*B)^-1
			// this is the real part of the result
			arm_mat_inverse_f32(&mat_interm_1, &mat_interm_2);

			for(uint8_t i = 0; i < N_MIC*N_MIC; i ++){
				vect_Rinv[f][2*i] = vect_interm_2[i];
			}

			// interm_3 = A*B^-1
			arm_mat_mult_f32(&mat_Rf_real, &mat_Rf_imag_inv, &mat_interm_1);

			// interm_4 = interm_3*A = A*B^-1*A
			arm_mat_mult_f32(&mat_interm_1, &mat_Rf_real, &mat_interm_2);

			// interm_3 = B+interm_4 = B + A*B^-1*A
			arm_mat_add_f32(&mat_Rf_imag, &mat_interm_2, &mat_interm_1);

			// interm_4 = interm_3^-1 =(B + A*B^-1*A)^-1
			// this is the imag part of the result
			arm_mat_inverse_f32(&mat_interm_1, &mat_interm_2);

			//arm_mat_inverse_f32(&matR[f], &matRinv[f]);

			for(uint8_t i = 0; i < N_MIC*N_MIC; i ++){
				vect_Rinv[f][2*i+1] = -vect_interm_2[i];
			}

			STOPCHRONO;
			time_bin_process = time_us;
		}

#ifdef FFT_FIND_PEAK
		/* Calculating the magnitude at each bin */
		arm_cmplx_mag_f32(left_1, testOutput, FFTSIZE);

		/* Calculates maxValue and returns corresponding BIN value */
		testOutput[0] = 0;
		arm_max_f32(testOutput, FFTSIZE, &maxValue, &testIndex);

		processing = 0;

#endif

#endif