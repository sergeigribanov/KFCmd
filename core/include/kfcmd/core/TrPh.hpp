//////////////////////////////////////////////////////////
// This class has been automatically generated on
// Tue Dec 24 21:50:46 2019 by ROOT version 6.18/04
// from TTree tr_ph/Tree with the non-collinear events
// found on file: tr_ph_run021171.root
//////////////////////////////////////////////////////////

#ifndef __KFCMD_TRPH_HPP__
#define __KFCMD_TRPH_HPP__

#include <TChain.h>
#include <TFile.h>
#include <TROOT.h>

#include <string>

// Header file for the classes stored in the TTree if any.

namespace kfcmd {
  namespace core {
    class TrPh {
    public:
      TTree *fChain;   //! pointer to the analyzed TTree or TChain
      Int_t fCurrent;  //! current Tree number in a TChain

      // Fixed size dimensions of array or collections stored in the TTree if any.

      // Declaration of leaf types
      Float_t ebeam;
      Float_t emeas;
      Float_t demeas;
      Float_t emeas0;
      Float_t demeas0;
      Float_t xbeam;
      Float_t ybeam;
      Int_t runnum;
      Int_t finalstate_id;
      Int_t evnum;
      Int_t trigbits;
      Int_t trigmchs;
      Float_t trigtime;
      Float_t time;
      Float_t dcfittime;
      Float_t anttime;
      Float_t mutime;
      Int_t is_coll;
      Int_t is_bhabha;
      Int_t nt_total;
      Float_t ecaltot;
      Float_t ecalneu;
      Float_t z0;
      Int_t nt;
      Int_t it[2];
      Int_t tnhit[20];                 //[nt]
      Float_t tlength[20];             //[nt]
      Float_t tphi[20];                //[nt]
      Float_t tth[20];                 //[nt]
      Float_t tptot[20];               //[nt]
      Float_t tphiv[20];               //[nt]
      Float_t tthv[20];                //[nt]
      Float_t tptotv[20];              //[nt]
      Float_t trho[20];                //[nt]
      Float_t tdedx[20];               //[nt]
      Float_t tz[20];                  //[nt]
      Float_t tt0[20];                 //[nt]
      Float_t tant[20];                //[nt]
      Float_t tchi2r[20];              //[nt]
      Float_t tchi2z[20];              //[nt]
      Float_t tchi2ndf[20];            //[nt]
      Int_t tcharge[20];               //[nt]
      Float_t ten[20];                 //[nt]
      Float_t tfc[20];                 //[nt]
      Float_t tenlxe[20];              //[nt]
      Float_t tlengthlxe[20];          //[nt]
      Float_t tencsi[20];              //[nt]
      Float_t tenbgo[20];              //[nt]
      Float_t terr[20][3][3];          //[nt]
      Float_t terr0[20][6][6];         //[nt]
      Float_t txyzatlxe[20][3];        //[nt]
      Int_t tenconv[20];               //[nt]
      Int_t nph_total;
      Int_t nph;
      Float_t phen[50];               //[nph]
      Float_t phth[50];               //[nph]
      Float_t phphi[50];              //[nph]
      Float_t phrho[50];              //[nph]
      Float_t phen0[50];              //[nph]
      Float_t phth0[50];              //[nph]
      Float_t phphi0[50];             //[nph]
      Float_t pherr[50][3];           //[nph]
      Float_t phcsi[50];              //[nph]
      Float_t phbgo[50];              //[nph]
      Int_t phflag[50];               //[nph]
      Int_t phfc[50];                 //[nph]
      Int_t nsim;
      Int_t simtype[50];     //[nsim]
      Int_t simorig[50];     //[nsim]
      Float_t simmom[50];    //[nsim]
      Float_t simphi[50];    //[nsim]
      Float_t simtheta[50];  //[nsim]
      Float_t simvtx[50];    //[nsim]
      Float_t simvty[50];    //[nsim]
      Float_t simvtz[50];    //[nsim]

      // List of branches
      TBranch *b_ebeam;            //!
      TBranch *b_emeas;            //!
      TBranch *b_demeas;           //!
      TBranch *b_emeas0;           //!
      TBranch *b_demeas0;          //!
      TBranch *b_xbeam;            //!
      TBranch *b_ybeam;            //!
      TBranch *b_runnum;           //!
      TBranch *b_finalstate_id;    //!
      TBranch *b_evnum;            //!
      TBranch *b_trigbits;         //!
      TBranch *b_trigmchs;         //!
      TBranch *b_trigtime;         //!
      TBranch *b_time;             //!
      TBranch *b_dcfittime;        //!
      TBranch *b_anttime;          //!
      TBranch *b_mutime;           //!
      TBranch *b_is_coll;          //!
      TBranch *b_is_bhabha;        //!
      TBranch *b_nt_total;         //!
      TBranch *b_ecaltot;          //!
      TBranch *b_ecalneu;          //!
      TBranch *b_z0;               //!
      TBranch *b_nt;               //!
      TBranch *b_it;               //!
      TBranch *b_tnhit;            //!
      TBranch *b_tlength;          //!
      TBranch *b_tphi;             //!
      TBranch *b_tth;              //!
      TBranch *b_tptot;            //!
      TBranch *b_tphiv;            //!
      TBranch *b_tthv;             //!
      TBranch *b_tptotv;           //!
      TBranch *b_trho;             //!
      TBranch *b_tdedx;            //!
      TBranch *b_tz;               //!
      TBranch *b_tt0;              //!
      TBranch *b_tant;             //!
      TBranch *b_tchi2r;           //!
      TBranch *b_tchi2z;           //!
      TBranch *b_tchi2ndf;         //!
      TBranch *b_tcharge;          //!
      TBranch *b_ten;              //!
      TBranch *b_tfc;              //!
      TBranch *b_tenlxe;           //!
      TBranch *b_tlengthlxe;       //!
      TBranch *b_tencsi;           //!
      TBranch *b_tenbgo;           //!
      TBranch *b_terr;             //!
      TBranch *b_terr0;            //!
      TBranch *b_txyzatlxe;        //!
      TBranch *b_tenconv;          //!
      TBranch *b_nph_total;        //!
      TBranch *b_nph;              //!
      TBranch *b_phen;             //!
      TBranch *b_phth;             //!
      TBranch *b_phphi;            //!
      TBranch *b_phrho;            //!
      TBranch *b_phen0;            //!
      TBranch *b_phth0;            //!
      TBranch *b_phphi0;           //!
      TBranch *b_pherr;            //!
      TBranch *b_phcsi;            //!
      TBranch *b_phbgo;            //!
      TBranch *b_phflag;           //!
      TBranch *b_phfc;             //!
      TBranch *b_nsim;             //!
      TBranch *b_simtype;          //!
      TBranch *b_simorig;          //!
      TBranch *b_simmom;           //!
      TBranch *b_simphi;           //!
      TBranch *b_simtheta;         //!
      TBranch *b_simvtx;           //!
      TBranch *b_simvty;           //!
      TBranch *b_simvtz;           //!

      TrPh(TTree *tree = 0);
      virtual ~TrPh();
      virtual Int_t Cut(Long64_t entry);
      virtual Int_t GetEntry(Long64_t entry);
      virtual Long64_t LoadTree(Long64_t entry);
      virtual void Init(TTree *tree);
      virtual void Loop(const std::string &, double magneticField) = 0;
      virtual Bool_t Notify();
      virtual void Show(Long64_t entry = -1);
    };
  } // namespace core
}  // namespace kfcmd
#endif
