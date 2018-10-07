import sys
import ROOT
from ROOT import TFile, TTree
import pickle
import os.path
import numpy as np
import numpy.ma as ma
import matplotlib.pyplot as plt


# -- input root file
#infile = "run26_beam25k_5s_rtrig100.root"
infile = sys.argv[1]
# pickle file to cache results
pklfile = infile + "tmp.pickle"

# suffix to add to the saved figures
#outname = "tmp_figs"
outname = infile;

new_tree_format = True # Set to false for old format/plots

make_type_dist = True
make_evt_time = True
make_freq_plot = True # Use the available date to calculate an average frequency 
make_channel_dist = True # Use the available data to show number of signals


limit_time_sample = 1  # Time sample in seconds

def pickle_data(input="ctb_pds_output_3.root",output="ctb_pds_output_3.pickle"):
  data_type = []
  data_ts = []
  data_payload = []
  data_beam = []
  data_crt = []
  fin = TFile.Open(input)
    
  for event in fin.data:
    data_ts.append(event.timestamp)
    data_payload.append(event.payload)
    if new_tree_format:
      data_type.append(event.word_type)
      data_beam.append(event.beam_status)
      data_crt.append(event.crt_status)
    else:
      data_type.append(event.word_type)
      data_beam.append(0)
      data_crt.append(0)

  #for event in fin.data:
  #  evt = {"type":event.word_type,"ts":event.timestamp,"payload":event.payload, "bi":event.beam}
  #  data.append(evt)

  ## Now picke the data
  pfile = open(output,'wb')
  #pickle.dump(data,pfile)
  pickle.dump(data_type,pfile)
  pickle.dump(data_ts,pfile)
  pickle.dump(data_payload,pfile)
  pickle.dump(data_beam,pfile)
  pickle.dump(data_crt,pfile)

  pfile.close()
  fin.Close()

def load_pickle(input="ctb_pds_output_3.pickle"):
  pfile = open(input,'r')
  data = pickle.load(pfile)
  pfile.close()
  return data

def count_of_type(data_types,data_words,type='HLT',mask=0x2):
  print('Determining if there are any {0} present'.format(type))
  types = ma.masked_not_equal(data_types,mask)
  ntp = types.count()

  print('Estimated  {0} words of type [{1}]. Distermining how many are distinct (bits)'.format(ntp,type))
  words_masked = ma.array(data_words,mask=types.mask)
  #ts_masked = ma.array(data_ts,mask=types.mask)

  # Reduce the masked datasets
  words_compressed = words_masked.compressed()
  #ts_compressed = ts_masked.compressed()

  # Now find how many different bits are we dealing with
#  bits = [0 for i in np.arange(61)] 
#  bits_bin = []
#  for word in words_compressed:
#    for bit in np.arange(61):
#      if word & (0x1 << bit):
#        bits[bit] = 1
#        if bit not in bits_bin:
#          bits_bin.append(bit)

  # Now find how many different bits are we dealing with
  bits = [0 for i in np.arange(61)] 
  bits_bin = []
  crt_num_words = 100
  for word in words_compressed:
    if (type == 'CRT'):
        if (crt_num_words > 0 ):
            #print word
            crt_num_words -= 1
 
    for bit in np.arange(61):
      if word & (0x1 << bit):
        bits[bit] = 1
        if bit not in bits_bin:
          bits_bin.append(bit)
  # bits holds effectively an OR of all bits asserted in all words
  # bits_bin holds the position of each bit in the word
  # The rest is actually superfluous
  return bits,bits_bin

def main():
  data = None
  if (not os.path.isfile(pklfile)):
    pickle_data(input=infile,output=pklfile)


  pfile = open(pklfile,'r')

  data_types = pickle.load(pfile)
  data_ts = pickle.load(pfile)
  data_payload = pickle.load(pfile)
  data_beam = pickle.load(pfile)
  data_crt = pickle.load(pfile)
  pfile.close()

  #data = load_pickle(input=pklfile)
  ## Now build the plots
  print('data loaded')
  # We'll use two separate gridspecs to have different margins, hspace, etc
  gs_type = plt.GridSpec(7, 5, top=0.93, bottom=0.20)
  gs_base = plt.GridSpec(7, 5, top=0.93, bottom=0.17)
  gs_wide = plt.GridSpec(25, 5, left=0.04,right=0.98,top=0.93)
  gs_ultra_wide = plt.GridSpec(25, 10, left=0.04,right=0.98,top=0.93)
  ## First make a histogram of word types
  ##
  #types = [entry['type'] for entry in data]
  #tstamps = [entry['ts'] for entry in data]
  #words = [entry['payload'] for entry in data]


  if limit_time_sample > 0:
    # Copy the timestamps until that number of seconds is past
    # THen remake the other arrays with the corresponding number of entries
    startts = data_ts[0]
    t_elapsed = 0
    cc = 0
    while t_elapsed < limit_time_sample:
      cc+=1
      t_elapsed = (data_ts[cc]-startts)*20.0/1e9

    print('After applying time sample a total of {0} tstamps are retained'.format(cc))
    data_types = data_types[:cc]
    data_ts = data_ts[:cc]
    data_payload = data_payload[:cc]
    data_beam = data_beam[:cc]
    data_crt = data_crt[:cc]

  if make_type_dist:
    ## The first is a simple histogram of the word types

    labels = ['feedback','LLT','HLT','ch_status','unused','unused','unused','timestamp']
    xx = np.arange(len(labels))
    print xx
    yy = np.bincount(data_types)
    if len(yy) < len(xx):
      yy = np.append(yy,[[0]]*(len(xx)-len(yy)))
    print yy
    fig = plt.figure(figsize=(7,5))
    ax = fig.add_subplot(gs_type[0:])
    ax.bar(xx,yy,color='b')
    ax.set_xlim([-0.5,len(labels)+0.5])
    ax.set_xticks(xx)
    ax.set_xticklabels(labels)

    for tick in ax.get_xticklabels():
      tick.set_rotation(45)
      print tick
    ax.set_ylabel('Number of words')
    ax.set_xlabel('Word type')
    ax.set_title('Word distribution by type')
    plt.savefig("type_dist{0}.pdf".format(outname),bbox_inches='tight')
    plt.savefig("type_dist{0}.png".format(outname),bbox_inches='tight')

  if make_evt_time:
    # set different colors for each set of positions
    ## -- Status words should all be blue
    ## -- triggers should all be red

    colors =[]
    lineoffsets = []
    linelengths = []
    labels = []

    # -- PDS formatting (black)
    # Calculate how many PDS bits we need ()
    pds_bits, pds_bits_bin=count_of_type(data_types,data_payload,type='PDS',mask=0x3)
    pds_bits_bin = np.sort(pds_bits_bin)
    n_pds = sum(pds_bits)
    pds_global_bins = []

    print('Estimated that we are in the presence of {0} different PDS channels'.format(n_pds))
    print('Estimated bits:  {0} '.format(pds_bits_bin))
    if n_pds > 0:
      for i,b in enumerate(pds_bits_bin):
        colors.append('black')
        if len(lineoffsets):
          lineoffsets = np.append(lineoffsets,lineoffsets[-1]+1)
          pds_global_bins.append(lineoffsets[-1])
        else:
          lineoffsets.append(0)
          pds_global_bins.append(0)

        linelengths.append(1)
        labels.append('PDS {0}'.format(b))

    ## Repeat for CRT : blue
    # Calculate how many CRT bits we need ()
    crt_bits, crt_bits_bin=count_of_type(data_types,data_crt,type='CRT',mask=0x3)
    crt_bits_bin = np.sort(crt_bits_bin)
    n_crt = sum(crt_bits)
    crt_global_bins = []

    print('Estimated that we are in the presence of {0} different CRT channels'.format(n_crt))
    print('Estimated bits:  {0} '.format(crt_bits_bin))
    if n_crt > 0:
      for i,b in enumerate(crt_bits_bin):
        colors.append('blue')
        if len(lineoffsets):
          lineoffsets = np.append(lineoffsets,lineoffsets[-1]+1)
          crt_global_bins.append(lineoffsets[-1])
        else:
          lineoffsets.append(0)
          crt_global_bins.append(0)

        linelengths.append(1)
        labels.append('CRT {0}'.format(b))

    # Beam : green
    beam_bits, beam_bits_bin=count_of_type(data_types,data_beam,type='Beam',mask=0x3)
    beam_bits_bin = np.sort(beam_bits_bin)
    n_beam = sum(beam_bits)
    beam_global_bins = []

    print('Estimated that we are in the presence of {0} different beam channels'.format(n_beam))
    print('Estimated bits:  {0} '.format(beam_bits_bin))
    if n_beam > 0:
      for i,b in enumerate(beam_bits_bin):
        colors.append('green')
        if len(lineoffsets):
          lineoffsets = np.append(lineoffsets,lineoffsets[-1]+1)
          beam_global_bins.append(lineoffsets[-1])
        else:
          lineoffsets.append(0)
          beam_global_bins.append(0)
        linelengths.append(1)
        labels.append('BI {0}'.format(b))


    # LLTs : orchid
    llt_bits, llt_bits_bin=count_of_type(data_types,data_payload,type='LLT',mask=0x1)
    llt_bits_bin = np.sort(llt_bits_bin)
    n_llt = sum(llt_bits)
    llt_global_bins = []

    print('Estimated that we are in the presence of {0} different LLTs'.format(n_llt))
    print('Estimated bits:  {0} '.format(llt_bits_bin))
    if n_llt > 0:
      for i,b in enumerate(llt_bits_bin):
        colors.append('orchid')
        if len(lineoffsets):
          lineoffsets = np.append(lineoffsets,lineoffsets[-1]+1)
          llt_global_bins.append(lineoffsets[-1])
        else:
          lineoffsets.append(0)
          llt_global_bins.append(0)
        linelengths.append(1)
        labels.append('LLT {0}'.format(b))

    # hlts : orchid
    hlt_bits, hlt_bits_bin=count_of_type(data_types,data_payload,type='HLT',mask=0x2)
    hlt_bits_bin = np.sort(hlt_bits_bin)
    n_hlt = sum(hlt_bits)
    hlt_global_bins = []

    print('Estimated that we are in the presence of {0} different HLTs'.format(n_hlt))
    print('Estimated bits:  {0} '.format(hlt_bits_bin))
    if n_hlt > 0:
      for i,b in enumerate(hlt_bits_bin):
        colors.append('orchid')
        if len(lineoffsets):
          lineoffsets = np.append(lineoffsets,lineoffsets[-1]+1)
          hlt_global_bins.append(lineoffsets[-1])
        else:
          lineoffsets.append(0)
          hlt_global_bins.append(0)
        linelengths.append(1)
        labels.append('HLT {0}'.format(b))




    max_ts = max(data_ts)
    min_ts = min(data_ts)

    #ntg = types.count(0x2)

    print('Estimated a total of {0} channels. Making the figure with (25,{1})'.format(len(labels),len(labels)/4))


    fig_size = (25,len(labels)/4 if len(labels) > 5 else 5)
    fig2 = plt.figure(figsize=fig_size)
    gs_ultra_wide = plt.GridSpec(fig_size[0], fig_size[1], left=0.04,right=0.98,top=0.93)

    # create a horizontal plot
    ax2 = fig2.add_subplot(gs_ultra_wide[0:])
    #
    # The data format is tricky. We want an array of 25 arrays of timestamps
    #
    #max_ts = -1
    #min_ts = 9e99
    data1 = [[] for i in np.arange(len(labels))]
    counter = [0]*len(labels)
    #for tp,ts,wd in [(e['type'],e['ts'],e['payload']) for e in data]:

    for tp,ts,wd1,wd2,wd3 in zip(data_types,data_ts,data_payload,data_crt,data_beam):
      # LLT
      if tp == 0x1:
        # Check which bins are active
        for i,b in enumerate(llt_bits_bin):
          if wd1 & (0x1 << b):
            # Cunundrum...which bit does this corresponds to in the global scheme?
            data1[llt_global_bins[i]].append(ts)
            counter[llt_global_bins[i]] += 1
      # HLT    
      elif tp == 0x2:
        # Check which bins are active
        for i,b in enumerate(hlt_bits_bin):
          # Skip these bits
          #  if b in [1]:
          #    continue
          if wd1 & (0x1 << b):
            # Cunundrum...which bit does this corresponds to in the global scheme?
            data1[hlt_global_bins[i]].append(ts)
            #print "This is the timestamp of the global trig: {1}"
            #print ts
            counter[hlt_global_bins[i]] += 1
      # Counter
      elif tp == 0x3:
        # In the case of counters we have to loop over all 3 types
        for i,b in enumerate(pds_bits_bin):
          if (wd1 & (0x1 << b)):
            #print 'Appending on ch {0} ts {1}'.format(i+1,ts)
            data1[pds_global_bins[i]].append(ts)
            counter[pds_global_bins[i]] += 1
        ## Repeat for the crt
        for i,b in enumerate(crt_bits_bin):
          if (wd2 & (0x1 << b)):
            #print 'Appending on ch {0} ts {1}'.format(i+1,ts)
            data1[crt_global_bins[i]].append(ts)
            counter[crt_global_bins[i]] += 1
        ## Repeat for the beam
        for i,b in enumerate(beam_bits_bin):
          if (wd3 & (0x1 << b)):
            #print 'Appending on ch {0} ts {1}'.format(i+1,ts)
            data1[beam_global_bins[i]].append(ts)
            counter[beam_global_bins[i]] += 1


    print('Counters : {0}'.format(counter))
    print('New data ordering done. Len {0})'.format(len(data1)))

    #yy = np.array(data1)
    #
    #print('New data ordering done. Shape {0} (len {1})'.format(yy.shape,len(yy)))

    for i in np.arange(len(data1)):
      print 'idx {0} : {1}'.format(i,len(data1[i]))


    print('Sizes : {0} {1} {2} {3}'.format(len(data1),len(colors),len(lineoffsets),len(linelengths)))
    ax2.eventplot(data1, colors=colors, lineoffsets=lineoffsets,
                linelengths=linelengths)
    ax2.set_title('Event time distribution by channel ({0:.2f} s)'.format((max_ts-min_ts)*20.0/1e9))
    ax2.set_yticks(lineoffsets)
    ax2.set_yticklabels(labels)
    ax2.set_xlabel('Timestamp (20ns ticks)')
    plt.savefig("channel_status_run{0}.pdf".format(outname),bbox_inches='tight')
    plt.savefig("channel_status_run{0}.png".format(outname),bbox_inches='tight')


    ##
    ## Plot number of words per channel
    ##
    if make_channel_dist:
      fig3  = plt.figure(figsize=(7,5))
      ax3   = fig3.add_subplot(gs_base[0:])
      xx    = np.arange(len(labels))
      yy    = counter

      ax3.bar(xx,yy,color='b')
      ax3.set_xlim([-0.5,len(labels)+0.5])
      ax3.set_xticks(xx)
      ax3.set_xticklabels(labels)

      for tick in ax3.get_xticklabels():
        tick.set_rotation(45)
        #print tick
      #ax.set_ylabel('Number of words')
      ax3.set_xlabel('Channel')
      ax3.set_title('Word distribution by channel ({0:.2f} s)'.format((max_ts-min_ts)*20.0/1e9))
      plt.savefig("channel_dist{0}.pdf".format(outname),bbox_inches='tight')
      plt.savefig("channel_dist{0}.png".format(outname),bbox_inches='tight')

    if make_freq_plot:

      fig4  = plt.figure(figsize=(7,5))
      ax4   = fig4.add_subplot(gs_base[0:])
      
      # labels=['TRIGGER']
      # for i in np.arange(24):
      #   labels.append('CH {0}'.format(i+1))

      xx    = np.arange(len(labels))
      # Run time in seconds
      timespan = (max_ts-min_ts)*20.0/1e9
      print('Run time span : {0:.2f} s'.format(timespan))
      yy    = np.divide(counter,timespan)

      ax4.bar(xx,yy,color='b')
      ax4.set_xlim([-0.5,len(labels)+0.5])
      ax4.set_xticks(xx)
      ax4.set_xticklabels(labels)

      for tick in ax4.get_xticklabels():
        tick.set_rotation(45)
        #print tick
      #ax.set_ylabel('Number of words')
      ax4.set_xlabel('Channel')
      ax4.set_ylabel('frequency (Hz)')
      ax4.set_title('Word frequency by channel ({0:.2f} s)'.format((max_ts-min_ts)*20.0/1e9))
      plt.savefig("channel_freq{0}.pdf".format(outname),bbox_inches='tight')
      plt.savefig("channel_freq{0}.png".format(outname),bbox_inches='tight')

  plt.show()



if __name__ == '__main__':
  main()
