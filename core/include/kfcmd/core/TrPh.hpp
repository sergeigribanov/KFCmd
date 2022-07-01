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
      Float_t psumch;
      Float_t psumnu;
      Float_t lumoff;
      Float_t lumofferr;
      Int_t nv_total;
      Int_t nv;
      Int_t vtrk[3];       //[nv]
      Int_t vind[3][10];   //[nv]
      Float_t vchi[3];     //[nv]
      Float_t vxyz[3][3];  //[nv]
      Int_t nt;
      Int_t it[2];
      Int_t tnhit[7];                 //[nt]
      Float_t tlength[7];             //[nt]
      Float_t tphi[7];                //[nt]
      Float_t tth[7];                 //[nt]
      Float_t tptot[7];               //[nt]
      Float_t tphiv[7];               //[nt]
      Float_t tthv[7];                //[nt]
      Float_t tptotv[7];              //[nt]
      Float_t trho[7];                //[nt]
      Float_t tdedx[7];               //[nt]
      Float_t tz[7];                  //[nt]
      Float_t tt0[7];                 //[nt]
      Float_t tant[7];                //[nt]
      Float_t tchi2r[7];              //[nt]
      Float_t tchi2z[7];              //[nt]
      Float_t tchi2ndf[7];            //[nt]
      Int_t tcharge[7];               //[nt]
      Float_t ten[7];                 //[nt]
      Float_t tfc[7];                 //[nt]
      Float_t tenlxe[7];              //[nt]
      Float_t tlengthlxe[7];          //[nt]
      Float_t tenslxe_layers[7][14];  //[nt]
      Float_t tencsi[7];              //[nt]
      Float_t tenbgo[7];              //[nt]
      Float_t tclth[7];               //[nt]
      Float_t tclphi[7];              //[nt]
      Float_t terr[7][3][3];          //[nt]
      Float_t terr0[7][5][5];         //[nt]
      Int_t tindlxe[7];               //[nt]
      Float_t tzcc[7][2];             //[nt]
      Float_t txyzatcl[7][3];         //[nt]
      Float_t txyzatlxe[7][3];        //[nt]
      Int_t tenconv[7];               //[nt]
      Int_t nks_total;
      Int_t nks;
      Int_t ksvind[4][2];      //[nks]
      Int_t kstype[4];         //[nks]
      Int_t ksfstatus[4];      //[nks]
      Float_t ksvchi[4];       //[nks]
      Float_t ksvxyz[4][3];    //[nks]
      Float_t ksminv[4];       //[nks]
      Float_t ksalign[4];      //[nks]
      Float_t kstlen[4];       //[nks]
      Float_t ksdpsi[4];       //[nks]
      Float_t kslen[4];        //[nks]
      Float_t ksz0[4];         //[nks]
      Float_t ksphi[4];        //[nks]
      Float_t ksth[4];         //[nks]
      Float_t ksptot[4];       //[nks]
      Float_t kspiphi[4][2];   //[nks]
      Float_t kspith[4][2];    //[nks]
      Float_t kspipt[4][2];    //[nks]
      Float_t kserr[4][3][3];  //[nks]
      Int_t ntlxe_total;
      Int_t ntlxe;
      Int_t ntlxelayers[10];           //[ntlxe]
      Int_t tlxenhit[10];              //[ntlxe]
      Float_t tlxelength[10];          //[ntlxe]
      Float_t tlxededx[10];            //[ntlxe]
      Float_t tlxeir[10];              //[ntlxe]
      Float_t tlxeitheta[10];          //[ntlxe]
      Float_t tlxeiphi[10];            //[ntlxe]
      Float_t tlxevtheta[10];          //[ntlxe]
      Float_t tlxevphi[10];            //[ntlxe]
      Float_t tlxechi2[10];            //[ntlxe]
      Float_t tlxesen[10];             //[ntlxe]
      Float_t tlxesen_layers[10][14];  //[ntlxe]
      Int_t nph_total;
      Int_t nph;
      Float_t phen[10];               //[nph]
      Float_t phth[10];               //[nph]
      Float_t phphi[10];              //[nph]
      Float_t phrho[10];              //[nph]
      Float_t phrad[10];              //[nph]
      Float_t phen0[10];              //[nph]
      Float_t phth0[10];              //[nph]
      Float_t phphi0[10];             //[nph]
      Float_t phlxe[10];              //[nph]
      Float_t phslxe_layers[10][14];  //[nph]
      Float_t pherr[10][3];           //[nph]
      Float_t phcsi[10];              //[nph]
      Float_t phbgo[10];              //[nph]
      Int_t phflag[10];               //[nph]
      Int_t phconv[10];               //[nph]
      Int_t phfc[10];                 //[nph]
      Int_t nzcs_total;
      Int_t nzcs;
      Int_t zcsch[15];      //[nzcs]
      Int_t zcsstat[15];    //[nzcs]
      Float_t zcsamp[15];   //[nzcs]
      Float_t zcstime[15];  //[nzcs]
      Float_t zcsphi[15];   //[nzcs]
      Int_t nzcc_total;
      Int_t nzcc;
      Int_t zccl[20];      //[nzcc]
      Int_t zccns[20];     //[nzcc]
      Float_t zccamp[20];  //[nzcc]
      Int_t zcct[20];      //[nzcc]
      Float_t zccz[20];    //[nzcc]
      Int_t zccvalid[20];  //[nzcc]
      Int_t nant;
      Int_t antch[16];    //[nant]
      Float_t antt0[16];  //[nant]
      Float_t antt1[16];  //[nant]
      Float_t anta0[16];  //[nant]
      Float_t anta1[16];  //[nant]
      Int_t antst[16];    //[nant]
      Int_t nmu;
      Int_t much[8];    //[nmu]
      Float_t mut0[8];  //[nmu]
      Float_t mut1[8];  //[nmu]
      Float_t mut2[8];  //[nmu]
      Float_t mut3[8];  //[nmu]
      Float_t mua0[8];  //[nmu]
      Float_t mua1[8];  //[nmu]
      Float_t mua2[8];  //[nmu]
      Float_t mua3[8];  //[nmu]
      Int_t must[8];    //[nmu]
      Int_t nsim;
      Int_t simtype[6];     //[nsim]
      Int_t simorig[6];     //[nsim]
      Float_t simmom[6];    //[nsim]
      Float_t simphi[6];    //[nsim]
      Float_t simtheta[6];  //[nsim]
      Float_t simvtx[6];    //[nsim]
      Float_t simvty[6];    //[nsim]
      Float_t simvtz[6];    //[nsim]
      Int_t ncorr;
      Int_t idcorr[1];   //[ncorr]
      Int_t bitcorr[1];  //[ncorr]
      Int_t nbadbank;
      Int_t nbadbankg;
      Int_t nbadbanks[1];  //[nbadbankg]
      Int_t nlostbanks;
      Int_t ncorruptedbanks;

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
      TBranch *b_psumch;           //!
      TBranch *b_psumnu;           //!
      TBranch *b_lumoff;           //!
      TBranch *b_lumofferr;        //!
      TBranch *b_nv_total;         //!
      TBranch *b_nv;               //!
      TBranch *b_vtrk;             //!
      TBranch *b_vind;             //!
      TBranch *b_vchi;             //!
      TBranch *b_vxyz;             //!
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
      TBranch *b_tenslxe_layers;   //!
      TBranch *b_tencsi;           //!
      TBranch *b_tenbgo;           //!
      TBranch *b_tclth;            //!
      TBranch *b_tclphi;           //!
      TBranch *b_terr;             //!
      TBranch *b_terr0;            //!
      TBranch *b_tindlxe;          //!
      TBranch *b_tzcc;             //!
      TBranch *b_txyzatcl;         //!
      TBranch *b_txyzatlxe;        //!
      TBranch *b_tenconv;          //!
      TBranch *b_nks_total;        //!
      TBranch *b_nks;              //!
      TBranch *b_ksvind;           //!
      TBranch *b_kstype;           //!
      TBranch *b_ksfstatus;        //!
      TBranch *b_ksvchi;           //!
      TBranch *b_ksvxyz;           //!
      TBranch *b_ksminv;           //!
      TBranch *b_ksalign;          //!
      TBranch *b_kstlen;           //!
      TBranch *b_ksdpsi;           //!
      TBranch *b_kslen;            //!
      TBranch *b_ksz0;             //!
      TBranch *b_ksphi;            //!
      TBranch *b_ksth;             //!
      TBranch *b_ksptot;           //!
      TBranch *b_kspiphi;          //!
      TBranch *b_kspith;           //!
      TBranch *b_kspipt;           //!
      TBranch *b_kserr;            //!
      TBranch *b_ntlxe_total;      //!
      TBranch *b_ntlxe;            //!
      TBranch *b_ntlxelayers;      //!
      TBranch *b_tlxenhit;         //!
      TBranch *b_tlxelength;       //!
      TBranch *b_tlxededx;         //!
      TBranch *b_tlxeir;           //!
      TBranch *b_tlxeitheta;       //!
      TBranch *b_tlxeiphi;         //!
      TBranch *b_tlxevtheta;       //!
      TBranch *b_tlxevphi;         //!
      TBranch *b_tlxechi2;         //!
      TBranch *b_tlxesen;          //!
      TBranch *b_tlxesen_layers;   //!
      TBranch *b_nph_total;        //!
      TBranch *b_nph;              //!
      TBranch *b_phen;             //!
      TBranch *b_phth;             //!
      TBranch *b_phphi;            //!
      TBranch *b_phrho;            //!
      TBranch *b_phrad;            //!
      TBranch *b_phen0;            //!
      TBranch *b_phth0;            //!
      TBranch *b_phphi0;           //!
      TBranch *b_phlxe;            //!
      TBranch *b_phslxe_layers;    //!
      TBranch *b_pherr;            //!
      TBranch *b_phcsi;            //!
      TBranch *b_phbgo;            //!
      TBranch *b_phflag;           //!
      TBranch *b_phconv;           //!
      TBranch *b_phfc;             //!
      TBranch *b_nzcs_total;       //!
      TBranch *b_nzcs;             //!
      TBranch *b_zcsch;            //!
      TBranch *b_zcsstat;          //!
      TBranch *b_zcsamp;           //!
      TBranch *b_zcstime;          //!
      TBranch *b_zcsphi;           //!
      TBranch *b_nzcc_total;       //!
      TBranch *b_nzcc;             //!
      TBranch *b_zccl;             //!
      TBranch *b_zccns;            //!
      TBranch *b_zccamp;           //!
      TBranch *b_zcct;             //!
      TBranch *b_zccz;             //!
      TBranch *b_zccvalid;         //!
      TBranch *b_nant;             //!
      TBranch *b_antch;            //!
      TBranch *b_antt0;            //!
      TBranch *b_antt1;            //!
      TBranch *b_anta0;            //!
      TBranch *b_anta1;            //!
      TBranch *b_antst;            //!
      TBranch *b_nmu;              //!
      TBranch *b_much;             //!
      TBranch *b_mut0;             //!
      TBranch *b_mut1;             //!
      TBranch *b_mut2;             //!
      TBranch *b_mut3;             //!
      TBranch *b_mua0;             //!
      TBranch *b_mua1;             //!
      TBranch *b_mua2;             //!
      TBranch *b_mua3;             //!
      TBranch *b_must;             //!
      TBranch *b_nsim;             //!
      TBranch *b_simtype;          //!
      TBranch *b_simorig;          //!
      TBranch *b_simmom;           //!
      TBranch *b_simphi;           //!
      TBranch *b_simtheta;         //!
      TBranch *b_simvtx;           //!
      TBranch *b_simvty;           //!
      TBranch *b_simvtz;           //!
      TBranch *b_ncorr;            //!
      TBranch *b_idcorr;           //!
      TBranch *b_bitcorr;          //!
      TBranch *b_nbadbank;         //!
      TBranch *b_nbadbankg;        //!
      TBranch *b_nbadbanks;        //!
      TBranch *b_nlostbanks;       //!
      TBranch *b_ncorruptedbanks;  //!

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
