#include "kfcmd/core/TrPh.hpp"

kfcmd::core::TrPh::TrPh(TTree *tree) : fChain(0) {
  // if parameter tree is not specified (or zero), connect the file
  // used to generate this class and read the Tree.
  if (tree == 0) {
    TFile *f =
        (TFile *)gROOT->GetListOfFiles()->FindObject("tr_ph_run021171.root");
    if (!f || !f->IsOpen()) {
      f = new TFile("tr_ph_run021171.root");
    }
    f->GetObject("tr_ph", tree);
  }
  Init(tree);
}

kfcmd::core::TrPh::~TrPh() {
}

Int_t kfcmd::core::TrPh::GetEntry(Long64_t entry) {
  // Read contents of entry.
  if (!fChain) return 0;
  return fChain->GetEntry(entry);
}
Long64_t kfcmd::core::TrPh::LoadTree(Long64_t entry) {
  // Set the environment to read one entry
  if (!fChain) return -5;
  Long64_t centry = fChain->LoadTree(entry);
  if (centry < 0) return centry;
  if (fChain->GetTreeNumber() != fCurrent) {
    fCurrent = fChain->GetTreeNumber();
    Notify();
  }
  return centry;
}

void kfcmd::core::TrPh::Init(TTree *tree) {
  // The Init() function is called when the selector needs to initialize
  // a new tree or chain. Typically here the branch addresses and branch
  // pointers of the tree will be set.
  // It is normally not necessary to make changes to the generated
  // code, but the routine can be extended by the user if needed.
  // Init() will be called many times when running on PROOF
  // (once per file to be processed).

  // Set branch addresses and branch pointers
  if (!tree) return;
  fChain = tree;
  fCurrent = -1;
  fChain->SetMakeClass(1);

  fChain->SetBranchAddress("ebeam", &ebeam, &b_ebeam);
  fChain->SetBranchAddress("emeas", &emeas, &b_emeas);
  fChain->SetBranchAddress("demeas", &demeas, &b_demeas);
  fChain->SetBranchAddress("emeas0", &emeas0, &b_emeas0);
  fChain->SetBranchAddress("demeas0", &demeas0, &b_demeas0);
  fChain->SetBranchAddress("xbeam", &xbeam, &b_xbeam);
  fChain->SetBranchAddress("ybeam", &ybeam, &b_ybeam);
  fChain->SetBranchAddress("runnum", &runnum, &b_runnum);
  fChain->SetBranchAddress("finalstate_id", &finalstate_id, &b_finalstate_id);
  fChain->SetBranchAddress("evnum", &evnum, &b_evnum);
  fChain->SetBranchAddress("trigbits", &trigbits, &b_trigbits);
  fChain->SetBranchAddress("trigmchs", &trigmchs, &b_trigmchs);
  fChain->SetBranchAddress("trigtime", &trigtime, &b_trigtime);
  fChain->SetBranchAddress("time", &time, &b_time);
  fChain->SetBranchAddress("dcfittime", &dcfittime, &b_dcfittime);
  fChain->SetBranchAddress("anttime", &anttime, &b_anttime);
  fChain->SetBranchAddress("mutime", &mutime, &b_mutime);
  fChain->SetBranchAddress("is_coll", &is_coll, &b_is_coll);
  fChain->SetBranchAddress("is_bhabha", &is_bhabha, &b_is_bhabha);
  fChain->SetBranchAddress("nt_total", &nt_total, &b_nt_total);
  fChain->SetBranchAddress("ecaltot", &ecaltot, &b_ecaltot);
  fChain->SetBranchAddress("ecalneu", &ecalneu, &b_ecalneu);
  fChain->SetBranchAddress("z0", &z0, &b_z0);
  fChain->SetBranchAddress("nt", &nt, &b_nt);
  fChain->SetBranchAddress("it", it, &b_it);
  fChain->SetBranchAddress("tnhit", tnhit, &b_tnhit);
  fChain->SetBranchAddress("tlength", tlength, &b_tlength);
  fChain->SetBranchAddress("tphi", tphi, &b_tphi);
  fChain->SetBranchAddress("tth", tth, &b_tth);
  fChain->SetBranchAddress("tptot", tptot, &b_tptot);
  fChain->SetBranchAddress("tphiv", tphiv, &b_tphiv);
  fChain->SetBranchAddress("tthv", tthv, &b_tthv);
  fChain->SetBranchAddress("tptotv", tptotv, &b_tptotv);
  fChain->SetBranchAddress("trho", trho, &b_trho);
  fChain->SetBranchAddress("tdedx", tdedx, &b_tdedx);
  fChain->SetBranchAddress("tz", tz, &b_tz);
  fChain->SetBranchAddress("tt0", tt0, &b_tt0);
  fChain->SetBranchAddress("tant", tant, &b_tant);
  fChain->SetBranchAddress("tchi2r", tchi2r, &b_tchi2r);
  fChain->SetBranchAddress("tchi2z", tchi2z, &b_tchi2z);
  fChain->SetBranchAddress("tchi2ndf", tchi2ndf, &b_tchi2ndf);
  fChain->SetBranchAddress("tcharge", tcharge, &b_tcharge);
  fChain->SetBranchAddress("ten", ten, &b_ten);
  fChain->SetBranchAddress("tfc", tfc, &b_tfc);
  fChain->SetBranchAddress("tenlxe", tenlxe, &b_tenlxe);
  fChain->SetBranchAddress("tlengthlxe", tlengthlxe, &b_tlengthlxe);
  fChain->SetBranchAddress("tencsi", tencsi, &b_tencsi);
  fChain->SetBranchAddress("tenbgo", tenbgo, &b_tenbgo);
  fChain->SetBranchAddress("terr", terr, &b_terr);
  fChain->SetBranchAddress("terr0", terr0, &b_terr0);
  fChain->SetBranchAddress("txyzatlxe", txyzatlxe, &b_txyzatlxe);
  fChain->SetBranchAddress("tenconv", tenconv, &b_tenconv);
  fChain->SetBranchAddress("nph_total", &nph_total, &b_nph_total);
  fChain->SetBranchAddress("nph", &nph, &b_nph);
  fChain->SetBranchAddress("phen", phen, &b_phen);
  fChain->SetBranchAddress("phth", phth, &b_phth);
  fChain->SetBranchAddress("phphi", phphi, &b_phphi);
  fChain->SetBranchAddress("phrho", phrho, &b_phrho);
  fChain->SetBranchAddress("phen0", phen0, &b_phen0);
  fChain->SetBranchAddress("phth0", phth0, &b_phth0);
  fChain->SetBranchAddress("phphi0", phphi0, &b_phphi0);
  fChain->SetBranchAddress("pherr", pherr, &b_pherr);
  fChain->SetBranchAddress("phcsi", phcsi, &b_phcsi);
  fChain->SetBranchAddress("phbgo", phbgo, &b_phbgo);
  fChain->SetBranchAddress("phflag", phflag, &b_phflag);
  fChain->SetBranchAddress("phfc", phfc, &b_phfc);
  fChain->SetBranchAddress("nsim", &nsim, &b_nsim);
  fChain->SetBranchAddress("simtype", simtype, &b_simtype);
  fChain->SetBranchAddress("simorig", simorig, &b_simorig);
  fChain->SetBranchAddress("simmom", simmom, &b_simmom);
  fChain->SetBranchAddress("simphi", simphi, &b_simphi);
  fChain->SetBranchAddress("simtheta", simtheta, &b_simtheta);
  fChain->SetBranchAddress("simvtx", simvtx, &b_simvtx);
  fChain->SetBranchAddress("simvty", simvty, &b_simvty);
  fChain->SetBranchAddress("simvtz", simvtz, &b_simvtz);
  Notify();
}

Bool_t kfcmd::core::TrPh::Notify() {
  // The Notify() function is called when a new file is opened. This
  // can be either for a new TTree in a TChain or when when a new TTree
  // is started when using PROOF. It is normally not necessary to make changes
  // to the generated code, but the routine can be extended by the
  // user if needed. The return value is currently not used.

  return kTRUE;
}

void kfcmd::core::TrPh::Show(Long64_t entry) {
  // Print contents of entry.
  // If entry is not specified, print current entry
  if (!fChain) return;
  fChain->Show(entry);
}
Int_t kfcmd::core::TrPh::Cut(Long64_t entry) {
  // This function may be called from Loop.
  // returns  1 if entry is accepted.
  // returns -1 otherwise.
  return 1;
}
