#include "TrPh.hpp"

KFCmd::TrPh::TrPh(TTree *tree) : fChain(0) {
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

KFCmd::TrPh::~TrPh() {
}

Int_t KFCmd::TrPh::GetEntry(Long64_t entry) {
  // Read contents of entry.
  if (!fChain) return 0;
  return fChain->GetEntry(entry);
}
Long64_t KFCmd::TrPh::LoadTree(Long64_t entry) {
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

void KFCmd::TrPh::Init(TTree *tree) {
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
  fChain->SetBranchAddress("psumch", &psumch, &b_psumch);
  fChain->SetBranchAddress("psumnu", &psumnu, &b_psumnu);
  fChain->SetBranchAddress("lumoff", &lumoff, &b_lumoff);
  fChain->SetBranchAddress("lumofferr", &lumofferr, &b_lumofferr);
  fChain->SetBranchAddress("nv_total", &nv_total, &b_nv_total);
  fChain->SetBranchAddress("nv", &nv, &b_nv);
  fChain->SetBranchAddress("vtrk", vtrk, &b_vtrk);
  fChain->SetBranchAddress("vind", vind, &b_vind);
  fChain->SetBranchAddress("vchi", vchi, &b_vchi);
  fChain->SetBranchAddress("vxyz", vxyz, &b_vxyz);
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
  fChain->SetBranchAddress("tenslxe_layers", tenslxe_layers, &b_tenslxe_layers);
  fChain->SetBranchAddress("tencsi", tencsi, &b_tencsi);
  fChain->SetBranchAddress("tenbgo", tenbgo, &b_tenbgo);
  fChain->SetBranchAddress("tclth", tclth, &b_tclth);
  fChain->SetBranchAddress("tclphi", tclphi, &b_tclphi);
  fChain->SetBranchAddress("terr", terr, &b_terr);
  fChain->SetBranchAddress("terr0", terr0, &b_terr0);
  fChain->SetBranchAddress("tindlxe", tindlxe, &b_tindlxe);
  fChain->SetBranchAddress("tzcc", tzcc, &b_tzcc);
  fChain->SetBranchAddress("txyzatcl", txyzatcl, &b_txyzatcl);
  fChain->SetBranchAddress("txyzatlxe", txyzatlxe, &b_txyzatlxe);
  fChain->SetBranchAddress("tenconv", tenconv, &b_tenconv);
  fChain->SetBranchAddress("nks_total", &nks_total, &b_nks_total);
  fChain->SetBranchAddress("nks", &nks, &b_nks);
  fChain->SetBranchAddress("ksvind", ksvind, &b_ksvind);
  fChain->SetBranchAddress("kstype", kstype, &b_kstype);
  fChain->SetBranchAddress("ksfstatus", ksfstatus, &b_ksfstatus);
  fChain->SetBranchAddress("ksvchi", ksvchi, &b_ksvchi);
  fChain->SetBranchAddress("ksvxyz", ksvxyz, &b_ksvxyz);
  fChain->SetBranchAddress("ksminv", ksminv, &b_ksminv);
  fChain->SetBranchAddress("ksalign", ksalign, &b_ksalign);
  fChain->SetBranchAddress("kstlen", kstlen, &b_kstlen);
  fChain->SetBranchAddress("ksdpsi", ksdpsi, &b_ksdpsi);
  fChain->SetBranchAddress("kslen", kslen, &b_kslen);
  fChain->SetBranchAddress("ksz0", ksz0, &b_ksz0);
  fChain->SetBranchAddress("ksphi", ksphi, &b_ksphi);
  fChain->SetBranchAddress("ksth", ksth, &b_ksth);
  fChain->SetBranchAddress("ksptot", ksptot, &b_ksptot);
  fChain->SetBranchAddress("kspiphi", kspiphi, &b_kspiphi);
  fChain->SetBranchAddress("kspith", kspith, &b_kspith);
  fChain->SetBranchAddress("kspipt", kspipt, &b_kspipt);
  fChain->SetBranchAddress("kserr", kserr, &b_kserr);
  fChain->SetBranchAddress("ntlxe_total", &ntlxe_total, &b_ntlxe_total);
  fChain->SetBranchAddress("ntlxe", &ntlxe, &b_ntlxe);
  fChain->SetBranchAddress("ntlxelayers", ntlxelayers, &b_ntlxelayers);
  fChain->SetBranchAddress("tlxenhit", tlxenhit, &b_tlxenhit);
  fChain->SetBranchAddress("tlxelength", tlxelength, &b_tlxelength);
  fChain->SetBranchAddress("tlxededx", tlxededx, &b_tlxededx);
  fChain->SetBranchAddress("tlxeir", tlxeir, &b_tlxeir);
  fChain->SetBranchAddress("tlxeitheta", tlxeitheta, &b_tlxeitheta);
  fChain->SetBranchAddress("tlxeiphi", tlxeiphi, &b_tlxeiphi);
  fChain->SetBranchAddress("tlxevtheta", tlxevtheta, &b_tlxevtheta);
  fChain->SetBranchAddress("tlxevphi", tlxevphi, &b_tlxevphi);
  fChain->SetBranchAddress("tlxechi2", tlxechi2, &b_tlxechi2);
  fChain->SetBranchAddress("tlxesen", tlxesen, &b_tlxesen);
  fChain->SetBranchAddress("tlxesen_layers", tlxesen_layers, &b_tlxesen_layers);
  fChain->SetBranchAddress("nph_total", &nph_total, &b_nph_total);
  fChain->SetBranchAddress("nph", &nph, &b_nph);
  fChain->SetBranchAddress("phen", phen, &b_phen);
  fChain->SetBranchAddress("phth", phth, &b_phth);
  fChain->SetBranchAddress("phphi", phphi, &b_phphi);
  fChain->SetBranchAddress("phrho", phrho, &b_phrho);
  fChain->SetBranchAddress("phen0", phen0, &b_phen0);
  fChain->SetBranchAddress("phth0", phth0, &b_phth0);
  fChain->SetBranchAddress("phphi0", phphi0, &b_phphi0);
  fChain->SetBranchAddress("phlxe", phlxe, &b_phlxe);
  fChain->SetBranchAddress("phslxe_layers", phslxe_layers, &b_phslxe_layers);
  fChain->SetBranchAddress("pherr", pherr, &b_pherr);
  fChain->SetBranchAddress("phcsi", phcsi, &b_phcsi);
  fChain->SetBranchAddress("phbgo", phbgo, &b_phbgo);
  fChain->SetBranchAddress("phflag", phflag, &b_phflag);
  fChain->SetBranchAddress("phconv", phconv, &b_phconv);
  fChain->SetBranchAddress("phfc", phfc, &b_phfc);
  fChain->SetBranchAddress("nzcs_total", &nzcs_total, &b_nzcs_total);
  fChain->SetBranchAddress("nzcs", &nzcs, &b_nzcs);
  fChain->SetBranchAddress("zcsch", zcsch, &b_zcsch);
  fChain->SetBranchAddress("zcsstat", zcsstat, &b_zcsstat);
  fChain->SetBranchAddress("zcsamp", zcsamp, &b_zcsamp);
  fChain->SetBranchAddress("zcstime", zcstime, &b_zcstime);
  fChain->SetBranchAddress("zcsphi", zcsphi, &b_zcsphi);
  fChain->SetBranchAddress("nzcc_total", &nzcc_total, &b_nzcc_total);
  fChain->SetBranchAddress("nzcc", &nzcc, &b_nzcc);
  fChain->SetBranchAddress("zccl", zccl, &b_zccl);
  fChain->SetBranchAddress("zccns", zccns, &b_zccns);
  fChain->SetBranchAddress("zccamp", zccamp, &b_zccamp);
  fChain->SetBranchAddress("zcct", zcct, &b_zcct);
  fChain->SetBranchAddress("zccz", zccz, &b_zccz);
  fChain->SetBranchAddress("zccvalid", zccvalid, &b_zccvalid);
  fChain->SetBranchAddress("nant", &nant, &b_nant);
  fChain->SetBranchAddress("antch", antch, &b_antch);
  fChain->SetBranchAddress("antt0", antt0, &b_antt0);
  fChain->SetBranchAddress("antt1", antt1, &b_antt1);
  fChain->SetBranchAddress("anta0", anta0, &b_anta0);
  fChain->SetBranchAddress("anta1", anta1, &b_anta1);
  fChain->SetBranchAddress("antst", antst, &b_antst);
  fChain->SetBranchAddress("nmu", &nmu, &b_nmu);
  fChain->SetBranchAddress("much", much, &b_much);
  fChain->SetBranchAddress("mut0", mut0, &b_mut0);
  fChain->SetBranchAddress("mut1", mut1, &b_mut1);
  fChain->SetBranchAddress("mut2", mut2, &b_mut2);
  fChain->SetBranchAddress("mut3", mut3, &b_mut3);
  fChain->SetBranchAddress("mua0", mua0, &b_mua0);
  fChain->SetBranchAddress("mua1", mua1, &b_mua1);
  fChain->SetBranchAddress("mua2", mua2, &b_mua2);
  fChain->SetBranchAddress("mua3", mua3, &b_mua3);
  fChain->SetBranchAddress("must", must, &b_must);
  fChain->SetBranchAddress("nsim", &nsim, &b_nsim);
  fChain->SetBranchAddress("simtype", simtype, &b_simtype);
  fChain->SetBranchAddress("simorig", simorig, &b_simorig);
  fChain->SetBranchAddress("simmom", simmom, &b_simmom);
  fChain->SetBranchAddress("simphi", simphi, &b_simphi);
  fChain->SetBranchAddress("simtheta", simtheta, &b_simtheta);
  fChain->SetBranchAddress("simvtx", simvtx, &b_simvtx);
  fChain->SetBranchAddress("simvty", simvty, &b_simvty);
  fChain->SetBranchAddress("simvtz", simvtz, &b_simvtz);
  fChain->SetBranchAddress("ncorr", &ncorr, &b_ncorr);
  fChain->SetBranchAddress("idcorr", &idcorr, &b_idcorr);
  fChain->SetBranchAddress("bitcorr", &bitcorr, &b_bitcorr);
  fChain->SetBranchAddress("nbadbank", &nbadbank, &b_nbadbank);
  fChain->SetBranchAddress("nbadbankg", &nbadbankg, &b_nbadbankg);
  fChain->SetBranchAddress("nbadbanks", &nbadbanks, &b_nbadbanks);
  fChain->SetBranchAddress("nlostbanks", &nlostbanks, &b_nlostbanks);
  fChain->SetBranchAddress("ncorruptedbanks", &ncorruptedbanks,
                           &b_ncorruptedbanks);
  Notify();
}

Bool_t KFCmd::TrPh::Notify() {
  // The Notify() function is called when a new file is opened. This
  // can be either for a new TTree in a TChain or when when a new TTree
  // is started when using PROOF. It is normally not necessary to make changes
  // to the generated code, but the routine can be extended by the
  // user if needed. The return value is currently not used.

  return kTRUE;
}

void KFCmd::TrPh::Show(Long64_t entry) {
  // Print contents of entry.
  // If entry is not specified, print current entry
  if (!fChain) return;
  fChain->Show(entry);
}
Int_t KFCmd::TrPh::Cut(Long64_t entry) {
  // This function may be called from Loop.
  // returns  1 if entry is accepted.
  // returns -1 otherwise.
  return 1;
}
