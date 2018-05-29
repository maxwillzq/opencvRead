int main()
{  
      CvLevMarq solver( nparams, 0, termCrit );
      double* param = solver.param->data.db;
      uchar* mask = solver.mask->data.ptr;
      //step 1: initialize param and mask
      ...
     //step 2: optimization
     for(;;)
    {
        const CvMat* _param = 0;
        CvMat *_JtJ = 0, *_JtErr = 0;
        double* _errNorm = 0;
        bool proceed = solver.updateAlt( _param, _JtJ, _JtErr, _errNorm ); //proceed will be STARTED, CALC_J,CHECK_ERR, DONE
        double *param = solver.param->data.db, *pparam = solver.prevParam->data.db;
        bool calcJ = solver.state == CvLevMarq::CALC_J;        
        A(0, 0) = param[0]; A(1, 1) = param[1]; A(0, 2) = param[2]; A(1, 2) = param[3];
        std::copy(param + 4, param + 4 + 14, k);

        if ( !proceed ) break;
        
        reprojErr = 0;

        for( i = 0, pos = 0; i < nimages; i++, pos += ni )
        {
          //Ji: Jocobian of intrinsic parameters
          //Je: Jocobian of extrinsic parameters
          //calculate Ji and Je for each image
         
            if( calcJ )
            {
                Mat JtJ(cvarrToMat(_JtJ)), JtErr(cvarrToMat(_JtErr));
                JtJ(Rect(0, 0, NINTRINSIC, NINTRINSIC)) += _Ji.t() * _Ji;
                JtJ(Rect(NINTRINSIC + i * 6, NINTRINSIC + i * 6, 6, 6)) = _Je.t() * _Je;
                JtJ(Rect(NINTRINSIC + i * 6, 0, 6, NINTRINSIC)) = _Ji.t() * _Je;

                JtErr.rowRange(0, NINTRINSIC) += _Ji.t() * _err;
                JtErr.rowRange(NINTRINSIC + i * 6, NINTRINSIC + (i + 1) * 6) = _Je.t() * _err;
            }
            double viewErr = norm(_err, NORM_L2SQR);
            reprojErr += viewErr;
        }
        if(_errNorm )
            *_errNorm = reprojErr;
    }
}
